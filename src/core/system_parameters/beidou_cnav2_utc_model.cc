/*
 * \file beidou_cnav2_utc_model.cc
 * \brief  Interface of a BEIDOU CNAV2 UTC MODEL storage
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
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

#include "beidou_cnav2_utc_model.h"

Beidou_Cnav2_Utc_Model::Beidou_Cnav2_Utc_Model()
{
    valid = false;
    d_tau_c = 0.0;
    d_tau_gps = 0.0;
    d_N_4 = 0.0;
    d_N_A = 0.0;
    d_B1 = 0.0;
    d_B2 = 0.0;
}


double Beidou_Cnav2_Utc_Model::utc_time(double beidou_time_corrected)
{
    double t_utc;

    // BEIDOU Time is relative to UTC Moscow, so we simply add its time difference
    t_utc = beidou_time_corrected + 3 * 3600 + d_tau_c;

    return t_utc;
}
