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

    // BDT-UTC Time Offset Parameters
    A_0UTC = 0.0;		//Bias coefficient of BDT time scale relative to UTC time scale [s]
	A_1UTC = 0.0;		//Drift coefficient of BDT time scale relative to UTC time scale [s/s]
	A_2UTC = 0.0;		//Drift rate coefficient of BDT time scale relative to UTC time scale [s/s^2]
	dt_LS = 0.0;		//Current of past leap second count [s]
	t_ot = 0.0;			//Reference time of week [s]
	WN_ot = 0.0;		//Reference week number [week]
	WN_LSF = 0.0;		//Leap second reference week number [week]
	DN = 0.0;			//Leap second reference day number [day]
	dt_LSF = 0.0;		//Current of future leap second count [s]

	// BDT-GNSS Time Offset Parameters
	GNSS_ID = 0.0;		//GNSS type identification [dimensionless]
	WN_0BGTO = 0.0;		//Reference week number [week]
	t_0BGTO = 0.0;		//Reference time of week [s]
	A_0BGTO = 0.0;		//Bias coefficient of BDT time scale relative to GNSS time scale [s]
	A_1BGTO = 0.0;		//Drift coefficient of BDT time scale relative to GNSS time scale [s/s]
	A_2BGTO = 0.0;		//Drift rate coefficient of BDT time scale relative to GNSS time scale [s/s^2]

	// Clock Correction Parameters
	t_oc = 0.0;
	a_0 = 0.0;
	a_1 = 0.0;
	a_2 = 0.0;
}


double Beidou_Cnav2_Utc_Model::utc_time(double beidou_time)
{
    double t_utc;

    // BEIDOU Time is relative to UTC Moscow, so we simply add its time difference
    t_utc = beidou_time + dt_LS;	// Adds the leap seconds that is broadcasted

    return t_utc;
}

boost::posix_time::ptime Beidou_Cnav2_Utc_Model::beidt_to_utc(const double offset_time, const double beidt2utc_corr) const
{
    /*
     * 1) DN is not in the past
     * t_UTC = (t_E - dt_UTC)mod86400
     * dt_UTC = dt_LS + A_0UTC + A_1UTC * (t_E - t_ot + 604800 * (WN - WN_ot)) + A_2UTC * (t_E - t_ot + 604800 * (WN - WN_ot))^2
     *
     * 2) user's present time falls within the time span which starts six hours prior to the leap second time and ends six hours after the leap second time
     * t_UTC = W mod(86400 + dt_LSF - dt_LS)
     * W = ((t_E - dt_UTC - 43200) mod86400) + 43200
     *
     * 3) DN is in the past
     * t_UTC = (t_E - dt_UTC)mod86400
     * dt_UTC = dt_LSF + A_0UTC + A_1UTC * (t_E - t_ot + 604800 * (WN - WN_ot)) + A_2UTC * (t_E - t_ot + 604800 * (WN - WN_ot))^2
     */
}


void Beidou_Cnav2_Utc_Model::beidt_to_gpst(double tod_offset, double beidt2utc_corr, double beidt2gpst_corr, double* wn, double* tow) const
{
	/*
	 * dT_systems = t_BD - t_GNSS = A_0BGTO + A_1BGTO*[t_BD - t_0BGTO + 604800*(WN - WN_BGTO)] + A_2BGTO*[t_BD - t_0BGTO +604800*(WN - WN_BGTO)]^2
	 */
}
