/*!
* \file beidou_b1c_dll_pll_tracking.cc
* \brief  Interface of an adapter of a DLL+PLL tracking loop block
* for BEIDOU B1C to a TrackingInterface
* \author Andrew Kamble, 2019. andrewkamble88@gmail.com
* \note Code added as part of GSoC 2019 program
*
* Code DLL + carrier PLL according to the algorithms described in:
* K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
* A Software-Defined GPS and Galileo Receiver. A Single-Frequency
* Approach, Birkhauser, 2007
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

#include "beidou_b1c_dll_pll_tracking.h"
#include "Beidou_B1C.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>


BeidouB1cDllPllTracking::BeidouB1cDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
   Dll_Pll_Conf trk_params = Dll_Pll_Conf();
   DLOG(INFO) << "role " << role;
   trk_params.SetFromConfiguration(configuration, role);
   //################# CONFIGURATION PARAMETERS ########################
   if (trk_params.extend_correlation_symbols < 1)
       {
           trk_params.extend_correlation_symbols = 1;
           std::cout << TEXT_RED << "WARNING: BEIDOU B1C. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
       }
   //!Warning:Beidou B1C does not have secondary data component
   /*else if (!track_pilot and extend_correlation_symbols > BEIDOU_B2ad_SECONDARY_CODE_LENGTH)
       {
           extend_correlation_symbols = BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
           std::cout << TEXT_RED << "WARNING: BEIDOU B2A. extend_correlation_symbols must be lower than 11 when tracking the data component. Coherent integration has been set to 10 symbols (10 ms)" << TEXT_RESET << std::endl;
       }*/
   if ((trk_params.extend_correlation_symbols > 1) and (trk_params.pll_bw_narrow_hz > trk_params.pll_bw_hz or trk_params.dll_bw_narrow_hz > trk_params.dll_bw_hz))
       {
           std::cout << TEXT_RED << "WARNING: BEIDOU B1C. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
       }
   trk_params.very_early_late_space_chips = 0.0;
   trk_params.very_early_late_space_narrow_chips = 0.0;
   trk_params.system = 'C';
   char sig_[3] = "C1";
   std::memcpy(trk_params.signal, sig_, 3);

   // ################# Make a GNU Radio Tracking block object ################
   if (trk_params.item_type == "gr_complex")
       {
           item_size_ = sizeof(gr_complex);
           tracking_ = dll_pll_veml_make_tracking(trk_params);
       }
   else
       {
           item_size_ = sizeof(gr_complex);
           LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
       }
   channel_ = 0;
   DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
   if (in_streams_ > 1)
       {
           LOG(ERROR) << "This implementation only supports one input stream";
       }
   if (out_streams_ > 1)
       {
           LOG(ERROR) << "This implementation only supports one output stream";
       }
}


BeidouB1cDllPllTracking::~BeidouB1cDllPllTracking() = default;


void BeidouB1cDllPllTracking::start_tracking()
{
   tracking_->start_tracking();
}


void BeidouB1cDllPllTracking::stop_tracking()
{
   tracking_->stop_tracking();
}


/*
* Set tracking channel unique ID
*/
void BeidouB1cDllPllTracking::set_channel(unsigned int channel)
{
   channel_ = channel;
   tracking_->set_channel(channel);
}


void BeidouB1cDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
   tracking_->set_gnss_synchro(p_gnss_synchro);
}


void BeidouB1cDllPllTracking::connect(gr::top_block_sptr top_block)
{
   if (top_block)
       { /* top_block is not null */
       };
   //nothing to connect, now the tracking uses gr_sync_decimator
}


void BeidouB1cDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
   if (top_block)
       { /* top_block is not null */
       };
   //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr BeidouB1cDllPllTracking::get_left_block()
{
   return tracking_;
}


gr::basic_block_sptr BeidouB1cDllPllTracking::get_right_block()
{
   return tracking_;
}