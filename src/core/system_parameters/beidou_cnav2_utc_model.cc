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
#include <cmath>

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

    t_utc = beidou_time + dt_LS;	// Adds the leap seconds that is broadcasted

    return t_utc;
}

double Beidou_Cnav2_Utc_Model::beidt_to_utc(Beidou_Cnav2_Ephemeris const&eph)
{
	double t_UTC;
	double dt_UTC;
	double W;

     // 1) DN is not in the past
	dt_UTC = dt_LS + A_0UTC + A_1UTC * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot)) + A_2UTC * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot)) * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot));
	t_UTC = fmod((eph.t_oe - dt_UTC),86400);

	 // 2) user's present time falls within the time span which starts six hours prior to the leap second time and ends six hours after the leap second time
	W = fmod((eph.t_oe - dt_UTC - 43200),86400) + 43200;
	t_UTC = fmod(W,(86400 + dt_LSF - dt_LS));

     // 3) DN is in the past
    dt_UTC = dt_LSF + A_0UTC + A_1UTC * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot)) + A_2UTC * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot)) * (eph.t_oe - t_ot + 604800 * (eph.WN - WN_ot));
    t_UTC = fmod((eph.t_oe - dt_UTC),86400);

    return t_UTC;
}

/*
template < typename t >
t square(t x) { return x * x; }
*/

double Beidou_Cnav2_Utc_Model::beidt_to_gpst(Beidou_Cnav2_Ephemeris const&eph, double beidou_time)
{
	double t_GNSS;
	
	 //dT_systems = t_BD - t_GNSS = A_0BGTO + A_1BGTO*[t_BD - t_0BGTO + 604800*(WN - WN_BGTO)] + A_2BGTO*square(t_BD - t_0BGTO +604800*(WN - WN_BGTO));
	 t_GNSS = beidou_time - (A_0BGTO + A_1BGTO*(beidou_time - t_0BGTO + 604800*(eph.WN - WN_0BGTO)) + A_2BGTO*(beidou_time - t_0BGTO +604800*(eph.WN - WN_0BGTO)))*(beidou_time - t_0BGTO +604800*(eph.WN - WN_0BGTO));
	 
	 return t_GNSS;
}
