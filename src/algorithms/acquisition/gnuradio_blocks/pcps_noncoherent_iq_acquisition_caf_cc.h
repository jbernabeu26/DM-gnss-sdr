/*!
 * \file pcps_nczp_acquisition_cc.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  data and pilot Signals performing a non-coherent integration of data and
 *  pilot signals with zero padding.
 * \author
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Marc Sales, 2014. marcsales92(at)gmail.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef PCPS_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_
#define PCPS_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_

#include "acq_conf.h"
#include "channel_fsm.h"
#include <armadillo>
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/gr_complex.h>     // for gr_complex
#include <gnuradio/thread/thread.h>  // for scoped_lock
#include <gnuradio/types.h>          // for gr_vector_const_void_star
#include <volk/volk_complex.h>       // for lv_16sc_t
#include <complex>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#if HAS_SPAN
#include <span>
namespace gsl = std;
#else
#include <gsl/gsl>
#endif

class pcps_noncoherent_iq_acquisition_caf_cc;

using pcps_noncoherent_iq_acquisition_caf_cc_sptr = boost::shared_ptr<pcps_noncoherent_iq_acquisition_caf_cc>;

pcps_noncoherent_iq_acquisition_caf_cc_sptr pcps_noncoherent_iq_make_acquisition_caf_cc(
    unsigned int sampled_ms,
    unsigned int max_dwells,
    unsigned int doppler_max, int64_t fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename,
    bool both_signal_components_,
    int CAF_window_hz_,
    int Zero_padding_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_noncoherent_iq_acquisition_caf_cc : public gr::block
{
public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_noncoherent_iq_acquisition_caf_cc();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks.
     * \param p_gnss_synchro Satellite information shared by the processing blocks.
     */
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
    {
        // require mutex with work function called by the scheduler
        gr::thread::scoped_lock lock(d_setlock);
        d_gnss_synchro = p_gnss_synchro;
    }

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const
    {
        return d_mag;
    }

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init();

    /*!
     * \brief Sets local code for PCPS acquisition algorithm.
     * \param code - Pointer to the PRN code.
     */
    void set_local_code(std::complex<float>* codeI, std::complex<float>* codeQ);

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    inline void set_active(bool active)
    {
        // require mutex with work function called by the scheduler
        gr::thread::scoped_lock lock(d_setlock);
        d_active = active;
    }

    /*!
     * \brief If set to 1, ensures that acquisition starts at the
     * first available sample.
     * \param state - int=1 forces start of acquisition
     */
    void set_state(int32_t state);

    /*!
     * \brief Set acquisition channel unique ID
     * \param channel - receiver channel.
     */
    inline void set_channel(uint32_t channel)
    {
        d_channel = channel;
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
    {
        d_channel_fsm = std::move(channel_fsm);
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm.
     * \param threshold - Threshold for signal detection (check \ref Navitec2012,
     * Algorithm 1, for a definition of this threshold).
     */
    inline void set_threshold(float threshold)
    {
        // require mutex with work function called by the scheduler
        gr::thread::scoped_lock lock(d_setlock);
        d_threshold = threshold;
    }

    /*!
     * \brief Set maximum Doppler grid search
     * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
     */
    inline void set_doppler_max(uint32_t doppler_max)
    {
        gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
        acq_parameters.doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(uint32_t doppler_step)
    {
        gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
        d_doppler_step = doppler_step;
    }

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend pcps_noncoherent_iq_acquisition_caf_cc_sptr
    pcps_noncoherent_iq_make_acquisition_caf_cc(const Acq_Conf& conf_);
    pcps_noncoherent_iq_acquisition_caf_cc(const Acq_Conf& conf_);

    std::weak_ptr<ChannelFsm> d_channel_fsm;
    int64_t d_fs_in;
    int d_samples_per_ms;
    int d_sampled_ms;
    int d_samples_per_code;
    unsigned int d_doppler_resolution;
    float d_threshold;
    std::string d_satellite_str;
    uint32_t d_doppler_max;
    uint32_t d_doppler_step;
    uint32_t d_max_dwells;
    uint32_t d_well_count;
    uint32_t d_fft_size;
    uint64_t d_sample_counter;
    gr_complex** d_grid_doppler_wipeoffs;
    unsigned int d_num_doppler_bins;
    gr_complex* d_fft_code_I_A;
    gr_complex* d_fft_code_I_B;
    gr_complex* d_fft_code_Q_A;
    gr_complex* d_fft_code_Q_B;
    gr_complex* d_inbuffer;
    std::shared_ptr<gr::fft::fft_complex> d_fft_if;
    std::shared_ptr<gr::fft::fft_complex> d_ifft;

    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;

    float d_input_power;
    float d_test_statistics;
    bool d_bit_transition_flag;
    std::ofstream d_dump_file;
    int d_state;
    bool d_dump;
    bool d_both_signal_components;
    int d_CAF_window_hz;
    float* d_CAF_vector;
    float* d_CAF_vector_I;
    float* d_CAF_vector_Q;
    unsigned int d_channel;
    std::string d_dump_filename;
    unsigned int d_buffer_count;
    unsigned int d_gr_stream_buffer;

    bool d_active;
    bool d_worker_active;
    bool d_cshort;
    bool d_step_two;
    bool d_use_CFAR_algorithm_flag;
    bool d_dump;
    int32_t d_state;
    int32_t d_positive_acq;
    uint32_t d_channel;
    uint32_t d_samplesPerChip;
    uint32_t d_doppler_step;
    uint32_t d_num_noncoherent_integrations_counter;
    uint32_t d_fft_size;
    uint32_t d_consumed_samples;
    uint32_t d_num_doppler_bins;
    uint32_t d_num_doppler_bins_step2;
    uint32_t d_dump_channel;
    uint32_t d_buffer_count;
    uint64_t d_sample_counter;
    int64_t d_dump_number;
    int64_t d_old_freq;
    float d_threshold;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    float d_doppler_center_step_two;
    std::string d_dump_filename;
    std::vector<std::vector<float>> d_magnitude_grid;
    std::vector<float> d_tmp_buffer_i;
    std::vector<float> d_tmp_buffer_q;
    std::vector<std::complex<float>> d_input_signal;
    std::vector<std::vector<std::complex<float>>> d_grid_doppler_wipeoffs;
    std::vector<std::vector<std::complex<float>>> d_grid_doppler_wipeoffs_step_two;
    std::vector<std::complex<float>> d_fft_code_i;
    std::vector<std::complex<float>> d_fft_code_q;
    std::vector<std::complex<float>> d_data_buffer;
    std::vector<lv_16sc_t> d_data_buffer_sc;
    std::shared_ptr<gr::fft::fft_complex> d_fft_if;
    std::shared_ptr<gr::fft::fft_complex> d_ifft;
    std::weak_ptr<ChannelFsm> d_channel_fsm;
    Acq_Conf acq_parameters;
    Gnss_Synchro* d_gnss_synchro;
    arma::fmat grid_;
    arma::fmat narrow_grid_;
    void update_local_carrier(gsl::span<gr_complex> carrier_vector, float freq);
    void update_grid_doppler_wipeoffs();
    void update_grid_doppler_wipeoffs_step2();
    void acquisition_core(uint64_t samp_count);
    void send_negative_acquisition();
    void send_positive_acquisition();
    void dump_results(int32_t effective_fft_size);
    bool is_fdma();
    bool start();
    float first_vs_second_peak_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);
    float max_to_input_power_statistic(uint32_t& indext, int32_t& doppler, float input_power, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);
};

#endif /* PCPS_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_ */
