/*!
* \file beidou_B1Cd_pcps_acquisition.h
* \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
*  BEIDOU B1C signals
* \authors <ul>
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

#ifndef GNSS_SDR_BEIDOU_B1C_PCPS_ACQUISITION_H_
#define GNSS_SDR_BEIDOU_B1C_PCPS_ACQUISITION_H_

#include "channel_fsm.h"
#include "complex_byte_to_float_x2.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/stream_to_vector.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstdint>
#include <string>


class ConfigurationInterface;

/*!
* \brief This class adapts a PCPS acquisition block to an
*  AcquisitionInterface for BeiDou B1C Signals
*/
class BeidouB1cPcpsAcquisition : public AcquisitionInterface
{
public:
   BeidouB1cPcpsAcquisition(const ConfigurationInterface* configuration,
       const std::string& role,
       unsigned int in_streams,
       unsigned int out_streams);

   virtual ~BeidouB1cPcpsAcquisition();

   inline std::string role() override
   {
       return role_;
   }

   /*!
    * \brief Returns "BEIDOU_B1C_PCPS_Acquisition"
    */
   inline std::string implementation() override
   {
       return "BEIDOU_B1C_PCPS_Acquisition";
   }

   size_t item_size() override
   {
       return item_size_;
   }

   void connect(gr::top_block_sptr top_block) override;
   void disconnect(gr::top_block_sptr top_block) override;
   gr::basic_block_sptr get_left_block() override;
   gr::basic_block_sptr get_right_block() override;

   /*!
    * \brief Set acquisition/tracking common Gnss_Synchro object pointer
    * to efficiently exchange synchronization data between acquisition and
    *  tracking blocks
    */
   void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

   /*!
    * \brief Set acquisition channel unique ID
    */
   inline void set_channel(unsigned int channel) override
   {
       channel_ = channel;
       acquisition_->set_channel(channel_);
   }

   /*!
     * \brief Set channel fsm associated to this acquisition instance
    */
   inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
   {
       channel_fsm_ = channel_fsm;
       acquisition_->set_channel_fsm(channel_fsm);
   }
   /*!
    * \brief Set statistics threshold of PCPS algorithm
    */
   void set_threshold(float threshold) override;

   /*!
    * \brief Set maximum Doppler off grid search
    */
   void set_doppler_max(unsigned int doppler_max) override;

   /*!
    * \brief Set Doppler steps for the grid search
    */
   void set_doppler_step(unsigned int doppler_step) override;

   /*!
    * \brief Initializes acquisition algorithm.
    */
   void init() override;

   /*!
    * \brief Sets local code for Galileo E1 PCPS acquisition algorithm.
    */
   void set_local_code() override;

   /*!
    * \brief Returns the maximum peak of grid search
    */
   signed int mag() override;

   /*!
    * \brief Restart acquisition algorithm
    */
   void reset() override;

   /*!
    * \brief If state = 1, it forces the block to start acquiring from the first sample
    */
   void set_state(int state) override;

   /*!
    * \brief Stop running acquisition
    */
   void stop_acquisition() override;

   /*!
    * \brief Sets the resampler latency to account it in the acquisition code delay estimation
    */

   void set_resampler_latency(uint32_t latency_samples) override;


private:
   ConfigurationInterface* configuration_;
   Acq_Conf acq_parameters_;
   pcps_acquisition_sptr acquisition_;
   gr::blocks::float_to_complex::sptr float_to_complex_;
   complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
   size_t item_size_;
   std::string item_type_;
   unsigned int vector_length_;
   unsigned int code_length_;
   unsigned int doppler_max_;
   unsigned int doppler_step_;
   unsigned int num_codes_;
   unsigned int in_streams_;
   unsigned int out_streams_;
   bool bit_transition_flag_;
   bool use_CFAR_algorithm_flag_;
   bool acq_pilot_;
   bool acq_iq_;
   unsigned int channel_;
   std::weak_ptr<ChannelFsm> channel_fsm_;
   float threshold_;
   unsigned int sampled_ms_;
   unsigned int max_dwells_;
   int64_t fs_in_;
   bool dump_;
   bool blocking_;
   std::string dump_filename_;
   std::vector<std::complex<float>> code_;
   Gnss_Synchro* gnss_synchro_;
   std::string role_;
   float calculate_threshold(float pfa);
};

#endif /* GNSS_SDR_BEIDOU_B1C_PCPS_ACQUISITION_H_ */