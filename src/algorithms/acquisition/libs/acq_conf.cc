/*!
 * \file acq_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algoritm.
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
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

#include "acq_conf.h"

Acq_Conf::Acq_Conf()
{
    /* PCPS acquisition configuration */
    sampled_ms = 0;
    max_dwells = 0;
    unsigned int doppler_max = 0;
    unsigned int num_doppler_bins_step2 = 0;
    float doppler_step2 = 0.0;
    long fs_in = 0;
    int samples_per_ms = 0;
    int samples_per_code = 0;
    bool bit_transition_flag = false;
    bool use_CFAR_algorithm_flag = false;
    bool dump = false;
    bool blocking = false;
    bool make_2_steps = false;
    std::string dump_filename = "";
    it_size = sizeof(char);
}
