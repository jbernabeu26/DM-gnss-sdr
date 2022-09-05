/*!
* \file beidou_b1cd pcps_acquisition.cc
* \brief Adapts a PCPS acquisition block to an Acquisition Interface for
*  BEIDOU B1C signals
* \authors <ul>
*           <li> Damian Miralles, 2018. dmiralles2009@gmail.com
*           <li> Andrew Kamble, 2019. andrewkamble88@gmail.com
*           <li> Joan Bernabeu, 2022. jbernabeu26@gmail.com
*          </ul>
* \note Code added as part of GSoC 2019 program
* \note Code updated as part of GSoC 2022 program
*
* -------------------------------------------------------------------------
*
* Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
* along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
 */

#include "beidou_b1c_pcps_acquisition.h"
#include "Beidou_B1C.h"
#include "acq_conf.h"
#include "beidou_b1c_signal_replica.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <algorithm>


#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl-lite.hpp>
namespace own = gsl;
#endif

BeidouB1cPcpsAcquisition::BeidouB1cPcpsAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : gnss_synchro_(nullptr),
                                role_(role),
                                threshold_(0.0),
                                channel_(0),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    acq_parameters_.ms_per_code = 10;
    acq_parameters_.SetFromConfiguration(configuration, role, BEIDOU_B1C_CODE_RATE_CPS, 10e6);

    LOG(INFO) << "role " << role;

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);
    fs_in_ = acq_parameters_.fs_in;
    item_type_ = acq_parameters_.item_type;
    item_size_ = acq_parameters_.it_size;

    num_codes_ = acq_parameters_.sampled_ms;
    code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(fs_in_) / (BEIDOU_B1C_CODE_RATE_CPS / BEIDOU_B1C_CODE_LENGTH_CHIPS)));
    vector_length_ = static_cast<unsigned int>(std::floor(acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms) * (acq_parameters_.bit_transition_flag ? 2.0 : 1.0));
    code_ = volk_gnsssdr::vector<std::complex<float>>(vector_length_ * num_codes_);
    acq_iq_ = acq_parameters_.acq_iq;
    acq_pilot_ = acq_parameters_.acq_pilot;

    if (acq_iq_)
    {
        acq_pilot_ = false;
    }

    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (item_type_ == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void BeidouB1cPcpsAcquisition::stop_acquisition()
{
    acquisition_->set_active(false);
}

void BeidouB1cPcpsAcquisition::set_threshold(float threshold)
{
    threshold_ = threshold;

    acquisition_->set_threshold(threshold_);
}


void BeidouB1cPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_->set_doppler_max(doppler_max_);
}


void BeidouB1cPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_->set_doppler_step(doppler_step_);
}


void BeidouB1cPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int BeidouB1cPcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void BeidouB1cPcpsAcquisition::init()
{
    acquisition_->init();
}


void BeidouB1cPcpsAcquisition::set_local_code()
{
    volk_gnsssdr::vector<std::complex<float>> code(code_length_);
    // Perform acquisition in Data + Pilot signal
    if (acq_iq_)
    {
        beidou_b1c_code_gen_complex_sampled_boc(code, gnss_synchro_->PRN, fs_in_);
    }
        // Perform acquisition in Pilot signal
    else if (acq_pilot_)
    {
        beidou_b1cp_code_gen_complex_sampled_boc_61_11(code, gnss_synchro_->PRN, fs_in_);
    }
        // Perform acquisition in Data signal
    else
    {
        beidou_b1cd_code_gen_complex_sampled_boc_11(code, gnss_synchro_->PRN, fs_in_);
    }
    //own::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < num_codes_; i++)
        {
            //std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
            std::copy(code.begin(), code.end(), code_.begin() + i * code_length_);
        }

    acquisition_->set_local_code(code_.data());

}

void BeidouB1cPcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


void BeidouB1cPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


void BeidouB1cPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            // nothing to connect
        }
    else if (item_type_ == "cbyte")
        {
            // Since a byte-based bds_b1c_acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void BeidouB1cPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cbyte")
        {
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr BeidouB1cPcpsAcquisition::get_left_block()
{
    if (item_type_ == "gr_complex" || item_type_ == "cshort")
        {
            return acquisition_;
        }
    if (item_type_ == "cbyte")
        {
            return cbyte_to_float_x2_;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
            return nullptr;
        }
}


gr::basic_block_sptr BeidouB1cPcpsAcquisition::get_right_block()
{
    return acquisition_;
}

void BeidouB1cPcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}