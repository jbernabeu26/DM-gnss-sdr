/*!
 * \file beidou_cnav2_navigation_message.cc
 * \brief  Implementation of a beidou cnav2 Data message decoder as described in beidou ICD (Edition 5.1)
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_beidou_eng_v5.1.pdf">beidou ICD</a>
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

#include "beidou_cnav2_navigation_message.h"
#include "gnss_satellite.h"
#include <glog/logging.h>


void beidou_cnav2_Navigation_Message::reset()
{
    // Satellite Identification
    i_satellite_PRN = 0;
    i_alm_satellite_slot_number = 0;  //!< SV Orbit Slot Number
    flag_update_slot_number = false;

    // Ephmeris Flags
    flag_all_ephemeris = false;
    flag_ephemeris_str_1 = false;
    flag_ephemeris_str_2 = false;
    flag_ephemeris_str_3 = false;
    flag_ephemeris_str_4 = false;

    // Almanac Flags
    flag_all_almanac = false;
    flag_almanac_str_6 = false;
    flag_almanac_str_7 = false;
    flag_almanac_str_8 = false;
    flag_almanac_str_9 = false;
    flag_almanac_str_10 = false;
    flag_almanac_str_11 = false;
    flag_almanac_str_12 = false;
    flag_almanac_str_13 = false;
    flag_almanac_str_14 = false;
    flag_almanac_str_15 = false;

    // UTC and System Clocks Flags
    flag_utc_model_valid = false;   //!< If set, it indicates that the UTC model parameters are filled
    flag_utc_model_str_5 = false;   //!< Clock info send in string 5 of navigation data
    flag_utc_model_str_15 = false;  //!< Clock info send in string 15 of frame 5 of navigation data

    // broadcast orbit 1
    flag_TOW_set = false;
    flag_TOW_new = false;

    flag_CRC_test = false;
    d_frame_ID = 0;
    d_string_ID = 0;
    i_channel_ID = 0;

    // Clock terms
    d_satClkCorr = 0.0;
    d_dtr = 0.0;
    d_satClkDrift = 0.0;

    // Data update information
    d_previous_tb = 0.0;
    for (unsigned int i = 0; i < BEIDOU_CA_NBR_SATS; i++)
        d_previous_Na[i] = 0.0;

    std::map<int, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    auto gnss_sat = Gnss_Satellite();
    std::string _system("beidou");
    //TODO SHould number of channels be hardcoded?
    for (unsigned int i = 1; i < 14; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


beidou_cnav2_Navigation_Message::beidou_cnav2_Navigation_Message()
{
    reset();
}


bool beidou_cnav2_Navigation_Message::CRC_test(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits)
{
    int sum_bits = 0;
    int sum_hamming = 0;
    int C1 = 0;
    int C2 = 0;
    int C3 = 0;
    int C4 = 0;
    int C5 = 0;
    int C6 = 0;
    int C7 = 0;
    int C_Sigma = 0;
    std::vector<int> string_bits(BEIDOU_CNAV2_STRING_BITS);

    //!< Populate data and hamming code vectors
    for (int i = 0; i < static_cast<int>(BEIDOU_CNAV2_STRING_BITS); i++)
        {
            string_bits[i] = static_cast<int>(bits[i]);
        }

    //!< Compute C1 term
    sum_bits = 0;
    for (int i = 0; i < static_cast<int>(BEIDOU_CNAV2_CRC_I_INDEX.size()); i++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_I_INDEX[i] - 1];
        }
    C1 = string_bits[0] ^ (sum_bits % 2);

    //!< Compute C2 term
    sum_bits = 0;
    for (int j = 0; j < static_cast<int>(BEIDOU_CNAV2_CRC_J_INDEX.size()); j++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_J_INDEX[j] - 1];
        }
    C2 = (string_bits[1]) ^ (sum_bits % 2);

    //!< Compute C3 term
    sum_bits = 0;
    for (int k = 0; k < static_cast<int>(BEIDOU_CNAV2_CRC_K_INDEX.size()); k++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_K_INDEX[k] - 1];
        }
    C3 = string_bits[2] ^ (sum_bits % 2);

    //!< Compute C4 term
    sum_bits = 0;
    for (int l = 0; l < static_cast<int>(BEIDOU_CNAV2_CRC_L_INDEX.size()); l++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_L_INDEX[l] - 1];
        }
    C4 = string_bits[3] ^ (sum_bits % 2);

    //!< Compute C5 term
    sum_bits = 0;
    for (int m = 0; m < static_cast<int>(BEIDOU_CNAV2_CRC_M_INDEX.size()); m++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_M_INDEX[m] - 1];
        }
    C5 = string_bits[4] ^ (sum_bits % 2);

    //!< Compute C6 term
    sum_bits = 0;
    for (int n = 0; n < static_cast<int>(BEIDOU_CNAV2_CRC_N_INDEX.size()); n++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_N_INDEX[n] - 1];
        }
    C6 = string_bits[5] ^ (sum_bits % 2);

    //!< Compute C7 term
    sum_bits = 0;
    for (int p = 0; p < static_cast<int>(BEIDOU_CNAV2_CRC_P_INDEX.size()); p++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_P_INDEX[p] - 1];
        }
    C7 = string_bits[6] ^ (sum_bits % 2);

    //!< Compute C_Sigma term
    sum_bits = 0;
    sum_hamming = 0;
    for (int q = 0; q < static_cast<int>(BEIDOU_CNAV2_CRC_Q_INDEX.size()); q++)
        {
            sum_bits += string_bits[BEIDOU_CNAV2_CRC_Q_INDEX[q] - 1];
        }
    for (int q = 0; q < 8; q++)
        {
            sum_hamming += string_bits[q];
        }
    C_Sigma = (sum_hamming % 2) ^ (sum_bits % 2);

    //!< Verification of the data
    // (a-i) All checksums (C1,...,C7 and C_Sigma) are equal to zero
    if ((C1 + C2 + C3 + C4 + C5 + C6 + C7 + C_Sigma) == 0)
        {
            return true;
        }
    // (a-ii) Only one of the checksums (C1,...,C7) is equal to zero but C_Sigma = 1
    else if (C_Sigma == 1 && C1 + C2 + C3 + C4 + C5 + C6 + C7 == 6)
        {
            return true;
        }
    else
        // All other conditions are assumed errors. TODO: Add correction for case B
        {
            return false;
        }
}


bool beidou_cnav2_Navigation_Message::read_navigation_bool(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    bool value;

    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int beidou_cnav2_Navigation_Message::read_navigation_unsigned(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


signed long int beidou_cnav2_Navigation_Message::read_navigation_signed(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    signed long int value = 0;
    signed long int sign = 0;
    int num_of_slices = parameter.size();
    // read the MSB and perform the sign extension
    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[0].first] == 1)
        {
            sign = -1;
        }
    else
        {
            sign = 1;
        }
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return (sign * value);
}


unsigned int beidou_cnav2_Navigation_Message::get_frame_number(unsigned int satellite_slot_number)
{
    unsigned int frame_ID = 0;

    if (satellite_slot_number >= 1 and satellite_slot_number <= 5)
        {
            frame_ID = 1;
        }
    else if (satellite_slot_number >= 6 and satellite_slot_number <= 10)
        {
            frame_ID = 2;
        }
    else if (satellite_slot_number >= 11 and satellite_slot_number <= 15)
        {
            frame_ID = 3;
        }
    else if (satellite_slot_number >= 16 and satellite_slot_number <= 20)
        {
            frame_ID = 4;
        }
    else if (satellite_slot_number >= 21 and satellite_slot_number <= 24)
        {
            frame_ID = 5;
        }
    else
        {
            LOG(WARNING) << "beidou cnav2: Invalid Satellite Slot Number";
            frame_ID = 0;
        }

    return frame_ID;
}


int beidou_cnav2_Navigation_Message::string_decoder(std::string frame_string)
{
    int J = 0;
    d_string_ID = 0;
    d_frame_ID = 0;

    // Unpack bytes to bits
    std::bitset<BEIDOU_CNAV2_STRING_BITS> string_bits(frame_string);

    // Perform data verification and exit code if error in bit sequence
    flag_CRC_test = CRC_test(string_bits);
    if (flag_CRC_test == false)
        return 0;

    // Decode all 15 string messages
    d_string_ID = static_cast<unsigned int>(read_navigation_unsigned(string_bits, STRING_ID));
    switch (d_string_ID)
        {
    // These should be changed according to the format outlined in the BEIDOU_B2a_CA.h

        case 1:
            //--- It is Type 10 -----------------------------------------------
        	/*


			*/
            break;

        case 2:
            //--- It is Type 11 -----------------------------------------------
        	/*
            if (flag_ephemeris_str_1 == true)
                {

                }
            */
            break;

        case 3:
            // --- It is Type 30 ----------------------------------------------
        	/*
            if (flag_ephemeris_str_2 == true)
                {

                }
            */
            break;

        case 4:
            // --- It is Type 31 ----------------------------------------------
        	/*
            if (flag_ephemeris_str_3 == true)
                {

                }
            */
            break;

        case 5:
            // --- It is Type 32 ----------------------------------------------
        	/*
            if (flag_ephemeris_str_4 == true)
                {

                }
            */
            break;

        case 6:
            // --- It is Type 33 ----------------------------------------------
        	/*

            */
            break;

        case 7:
            // --- It is Type 34 ----------------------------------------------
        	/*
            if (flag_almanac_str_6 == true)
                {

                }
            */
            break;

        case 8:
            // --- It is Type 40 ----------------------------------------------
        	/*

            */
            break;


        default:
            LOG(INFO) << "beidou cnav2: Invalid String ID of received. Received " << d_string_ID
                      << ", but acceptable range is from 10 11 30 31 32 33 34 40";

            break;
        }  // switch string ID

    return d_string_ID;
}


Beidou_Cnav2_Ephemeris beidou_cnav2_Navigation_Message::get_ephemeris()
{
    return cnav2_ephemeris;
}

/*
Beidou_Cnav2_Utc_Model beidou_cnav2_Navigation_Message::get_utc_model()
{
    return cnav2_utc_model;
}
*/

Beidou_Cnav2_Almanac beidou_cnav2_Navigation_Message::get_almanac(unsigned int satellite_slot_number)
{
    return cnav2_almanac[satellite_slot_number - 1];
}


bool beidou_cnav2_Navigation_Message::have_new_ephemeris()  //Check if we have a new ephemeris stored in the galileo navigation class
{
    bool new_eph = false;
    // We need to make sure we have received the ephemeris info plus the time info
    if ((flag_ephemeris_str_1 == true) and (flag_ephemeris_str_2 == true) and
        (flag_ephemeris_str_3 == true) and (flag_ephemeris_str_4 == true) and
        (flag_utc_model_str_5 == true))
    	/*
        {
            if (d_previous_tb != cnav2_ephemeris.d_t_b)
                {
                    flag_ephemeris_str_1 = false;  // clear the flag
                    flag_ephemeris_str_2 = false;  // clear the flag
                    flag_ephemeris_str_3 = false;  // clear the flag
                    flag_ephemeris_str_4 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    // Update the time of ephemeris information
                    d_previous_tb = cnav2_ephemeris.d_t_b;
                    DLOG(INFO) << "beidou cnav2 Ephemeris (1, 2, 3, 4) have been received and belong to the same batch" << std::endl;
                    new_eph = true;
                }
        }
        */

    return new_eph;
}


bool beidou_cnav2_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the galileo navigation class
{
    if (flag_utc_model_str_5 == true)
        {
            flag_utc_model_str_5 = false;  // clear the flag
            return true;
        }
    else
        return false;
}

/*
bool beidou_cnav2_Navigation_Message::have_new_almanac()  //Check if we have a new almanac data set stored in the galileo navigation class
{
    bool new_alm = false;
    if ((flag_almanac_str_6 == true) and (flag_almanac_str_7 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_str_6 = false;
                    flag_almanac_str_7 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_8 == true) and (flag_almanac_str_9 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_8 = false;
                    flag_almanac_str_9 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_10 == true) and (flag_almanac_str_11 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_10 = false;
                    flag_almanac_str_11 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_12 == true) and (flag_almanac_str_13 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_12 = false;
                    flag_almanac_str_13 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_14 == true) and (flag_almanac_str_15 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_14 = false;
                    flag_almanac_str_15 = false;
                    new_alm = true;
                }
        }

    return new_alm;
}
*/
