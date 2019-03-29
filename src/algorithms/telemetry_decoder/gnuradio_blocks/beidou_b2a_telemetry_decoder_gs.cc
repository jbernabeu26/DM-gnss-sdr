/*!
 * \file beidou_b2a_telemetry_decoder_gs.cc
 * \brief Implementation of a BeiDou B2a CNAV2 data decoder block
  * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "beidou_b2a_telemetry_decoder_gs.h"
#include "Beidou_B2a.h"
#include "beidou_cnav2_almanac.h"
#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_utc_model.h"
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstdlib>    // for abs
#include <exception>  // for exception
#include <iostream>   // for cout
#include <memory>     // for shared_ptr, make_shared


#define CRC_ERROR_LIMIT 8


beidou_b2a_telemetry_decoder_gs_sptr
beidou_b2a_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump)
{
    return beidou_b2a_telemetry_decoder_gs_sptr(new beidou_b2a_telemetry_decoder_gs(satellite, dump));
}


beidou_b2a_telemetry_decoder_gs::beidou_b2a_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("beidou_b2a_telemetry_decoder_gs",
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing BEIDOU B2a TELEMETRY DECODING";
    // Define the number of sampes per symbol. Notice that BEIDOU has 2 rates, !!!Change
    //one for the navigation data and the other for the preamble information
    d_samples_per_symbol = (BEIDOU_B2a_CODE_RATE_HZ / BEIDOU_B2a_CODE_LENGTH_CHIPS) / BEIDOU_B2a_SYMBOL_RATE_SPS;
    d_symbols_per_preamble = BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS;

    // set the preamble
    d_samples_per_preamble = BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS * d_samples_per_symbol;

    // preamble symbols to samples
    d_secondary_code_samples = static_cast<int32_t *>(volk_gnsssdr_malloc(BEIDOU_B2ad_SECONDARY_CODE_LENGTH * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    d_preamble_samples = static_cast<int32_t *>(volk_gnsssdr_malloc(d_samples_per_preamble * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    d_preamble_period_samples = BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS * d_samples_per_symbol;
    d_frame_length_symbols = BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS - BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS;

    // Setting samples of secondary code
    for (int32_t i = 0; i < BEIDOU_B2ad_SECONDARY_CODE_LENGTH; i++)
        {
            if (BEIDOU_B2ad_SECONDARY_CODE.at(i) == '1')
                {
                    d_secondary_code_samples[i] = 1;
                }
            else
                {
                    d_secondary_code_samples[i] = -1;
                }
        }

    // Setting samples of preamble code
    int32_t n = 0;
    for (int32_t i = 0; i < d_symbols_per_preamble; i++)
        {
            int32_t m = 0;
            if (BEIDOU_CNAV2_PREAMBLE.at(i) == '1')
                {
                    for (uint32_t j = 0; j < d_samples_per_symbol; j++)
                        {
                            d_preamble_samples[n] = d_secondary_code_samples[m];
                            n++;
                            m++;
                            m = m % BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
                        }
                }
            else
                {
                    for (uint32_t j = 0; j < d_samples_per_symbol; j++)
                        {
                            d_preamble_samples[n] = -d_secondary_code_samples[m];
                            n++;
                            m++;
                            m = m % BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
                        }
                }
        }

    d_frame_symbols = static_cast<double *>(volk_gnsssdr_malloc(d_frame_length_symbols * sizeof(double), volk_gnsssdr_get_alignment()));
    d_required_symbols = BEIDOU_CNAV2_FRAME_SYMBOLS * d_samples_per_symbol;
    d_symbol_history.set_capacity(d_required_symbols + 1);

    // Generic settings
    d_sample_counter = 0;
    d_stat = 0;
    d_preamble_index = 0;
    d_flag_frame_sync = false;
    d_TOW_at_current_symbol_ms = 0;
    Flag_valid_word = false;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
}


beidou_b2a_telemetry_decoder_gs::~beidou_b2a_telemetry_decoder_gs()
{
    volk_gnsssdr_free(d_preamble_samples);
    volk_gnsssdr_free(d_secondary_code_samples);
    volk_gnsssdr_free(d_frame_symbols);

    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


void beidou_b2a_telemetry_decoder_gs::decode_string(double *frame_symbols, int32_t frame_length)
{
    // 1. Transform from symbols to bits
    std::string data_bits;

    // we want data_bits = frame_symbols[24:24+288]
    for (uint32_t ii = 0; ii < (BEIDOU_CNAV2_DATA_BITS); ii++)
        {
            data_bits.push_back((frame_symbols[ii] > 0) ? ('1') : ('0'));
        }

    d_nav.string_decoder(data_bits);

    // 3. Check operation executed correctly
    if (d_nav.flag_crc_test == true)
        {
            LOG(INFO) << "BeiDou CNAV2 CRC correct in channel " << d_channel << " from satellite " << d_satellite;
        }
    else
        {
            LOG(INFO) << "BeiDou CNAV2 CRC error in channel " << d_channel << " from satellite " << d_satellite;
        }
    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Ephemeris> tmp_obj = std::make_shared<Beidou_Cnav2_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU CNAV2 Ephemeris have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << "New BEIDOU B2a CNAV2 message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << std::endl;
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Utc_Model> tmp_obj = std::make_shared<Beidou_Cnav2_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU CNAV2 UTC Model have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << "New BEIDOU B2a CNAV2 utc model message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << std::endl;
        }
    if (d_nav.have_new_iono() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav2_Iono> tmp_obj = std::make_shared<Beidou_Cnav2_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU CNAV2 Iono have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << "New BEIDOU B2a CNAV2 Iono message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << std::endl;
        }
    if (d_nav.have_new_almanac() == true)
        {
            unsigned int slot_nbr = d_nav.i_alm_satellite_PRN;
            std::shared_ptr<Beidou_Cnav2_Almanac> tmp_obj = std::make_shared<Beidou_Cnav2_Almanac>(d_nav.get_almanac(slot_nbr));
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU CNAV2 Almanac have been received in channel" << d_channel << " in slot number " << slot_nbr;
            std::cout << "New BEIDOU B2a CNAV2 almanac received in channel " << d_channel << " from satellite " << d_satellite << std::endl;
        }
}


void beidou_b2a_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void beidou_b2a_telemetry_decoder_gs::set_channel(int channel)
{
    d_channel = channel;
    LOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening Beidou TLM dump file. " << e.what();
                        }
                }
        }
}


int beidou_b2a_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t corr_value = 0;
    int32_t preamble_diff = 0;

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol;  //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol.Prompt_I);  //add new symbol to the symbol queue
    d_sample_counter++;                                   //count for the processed samples
    consume_each(1);

    d_flag_preamble = false;

    if (d_symbol_history.size() > d_required_symbols)
        {
            //******* preamble correlation ********
            for (int i = 0; i < d_samples_per_preamble; i++)
                {
                    if (d_symbol_history[i] < 0)  // symbols clipping
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                    else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                }
        }

    //******* frame sync ******************
    if (d_stat == 0)  //no preamble information
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    // Record the preamble sample stamp
                    d_preamble_index = d_sample_counter;
                    LOG(INFO) << "Preamble detection for BEIDOU B2a SAT " << this->d_satellite;
                    // Enter into frame pre-detection status
                    d_stat = 1;
                }
        }
    else if (d_stat == 1)  // possible preamble lock
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    //check preamble separation
                    preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    if (abs(preamble_diff - d_preamble_period_samples) == 0)
                        {
                            //try to decode frame
                            LOG(INFO) << "Starting BeiDou CNAV2 frame decoding for BeiDou B2a SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            d_stat = 2;
                        }
                    else
                        {
                            if (preamble_diff > d_preamble_period_samples)
                                {
                                    d_stat = 0;  // start again
                                }
                            DLOG(INFO) << "Failed BeiDou CNAV2 frame decoding for BeiDou B2a SAT " << this->d_satellite;
                        }
                }
        }
    else if (d_stat == 2)  // preamble acquired
        {
            if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_samples))
                {
                    //******* SAMPLES TO SYMBOLS *******
                    if (corr_value > 0)  //normal PLL lock
                        {
                            int k = 0;
                            for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                {
                                    d_frame_symbols[i] = 0;
                                    //integrate samples into symbols
                                    for (uint32_t m = 0; m < d_samples_per_symbol; m++)
                                        {
                                            // because last symbol of the preamble is just received now!
                                            d_frame_symbols[i] += static_cast<float>(d_secondary_code_samples[k]) * d_symbol_history.at(i * d_samples_per_symbol + d_samples_per_preamble + m);
                                            k++;
                                            k = k % BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
                                        }
                                }
                        }
                    else  //180 deg. inverted carrier phase PLL lock
                        {
                            int k = 0;
                            for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                {
                                    d_frame_symbols[i] = 0;
                                    //integrate samples into symbols
                                    for (uint32_t m = 0; m < d_samples_per_symbol; m++)
                                        {
                                            // because last symbol of the preamble is just received now!
                                            d_frame_symbols[i] -= static_cast<float>(d_secondary_code_samples[k]) * d_symbol_history.at(i * d_samples_per_symbol + d_samples_per_preamble + m);
                                            k++;
                                            k = k % BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
                                        }
                                }
                        }

                    //call the decoder
                    decode_string(d_frame_symbols, d_frame_length_symbols);
                    if (d_nav.flag_crc_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;               //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << "BeiDou CNAV2 frame sync found for SAT " << this->d_satellite;
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    LOG(INFO) << "BeiDou CNAV2 frame sync lost for SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                }
                        }
                }
        }

    // UPDATE GNSS SYNCHRO DATA
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_TOW_set == true)
        //update TOW at the preamble instant
        {
            if (d_nav.flag_TOW_10 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_10 = false;
                }
            else if (d_nav.flag_TOW_11 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_11 = false;
                }
            else if (d_nav.flag_TOW_30 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_30 = false;
                }
            else if (d_nav.flag_TOW_31 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_31 = false;
                }
            else if (d_nav.flag_TOW_32 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_32 = false;
                }
            else if (d_nav.flag_TOW_33 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_33 = false;
                }
            else if (d_nav.flag_TOW_34 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_34 = false;
                }
            else if (d_nav.flag_TOW_40 == true)
                {
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.cnav2_ephemeris.SOW * 1000.0);
                    d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * BEIDOU_B2ad_PERIOD_MS);
                    d_nav.flag_TOW_40 = false;
                }
            else  //if there is not a new preamble, we define the TOW of the current symbol
                {
                    d_TOW_at_current_symbol_ms += static_cast<uint32_t>(BEIDOU_B2ad_PERIOD_MS);
                    ;
                }
        }
    else  //if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol_ms += static_cast<uint32_t>(BEIDOU_B2ad_PERIOD_MS);
            ;
        }


    //if (d_flag_frame_sync == true and d_nav.flag_TOW_set==true and d_nav.flag_CRC_test == true)

    // if(d_nav.flag_GGTO_1 == true  and  d_nav.flag_GGTO_2 == true and  d_nav.flag_GGTO_3 == true and  d_nav.flag_GGTO_4 == true) //all GGTO parameters arrived
    //     {
    //         delta_t = d_nav.A_0G_10 + d_nav.A_1G_10 * (d_TOW_at_current_symbol - d_nav.t_0G_10 + 604800.0 * (fmod((d_nav.WN_0 - d_nav.WN_0G_10), 64)));
    //     }

    if (d_flag_frame_sync == true and d_nav.flag_TOW_set == true)
        {
            current_symbol.Flag_valid_word = true;
        }
    else
        {
            current_symbol.Flag_valid_word = false;
        }

    current_symbol.PRN = this->d_satellite.get_PRN();
    current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;

    if (d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
                {
                    double tmp_double;
                    unsigned long int tmp_ulong_int;
                    tmp_double = d_TOW_at_current_symbol_ms;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(unsigned long int));
                    tmp_double = 0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
                }
        }

    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_symbol;

    return 1;
}
