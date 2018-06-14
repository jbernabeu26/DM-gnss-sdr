/*!
 * \file beidou_cnav2_ephemeris.cc
 * \brief  Interface of a BEIDOU CNAV2 EPHEMERIS storage and orbital model functions
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
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

#include "beidou_cnav2_ephemeris.h"
#include "gnss_satellite.h"
#include "BEIDOU_B2a_CA.h"
#include <cmath>


Beidou_Cnav2_Ephemeris::Beidou_Cnav2_Ephemeris()
{

    t_oe = 0.0;				//!< Ephemeris reference time [s]
    SatType = 0.0;			//!< Satellite orbit type [dimensionless]
    d_A = 0.0;				//!< Semi-major axis difference at reference time [m]
    A_dot = 0.0;			//!< Change rate in semi-major axis [m/s]
    dn_0 = 0.0;				//!< Mean motion difference from computed value at reference time [pi/s]
    dn_0_dot = 0.0;			//!< Rate of mean motion difference from computed value at reference time [pi/s^2]
    M_0 = 0.0;				//!< Mean anomaly at reference time [pi]
    e = 0.0;				//!< Eccenticity [dimensionless]
    omega = 0.0;			//!< Argument of perigee [pi]
    Omega_0 = 0.0;			//!< Longitude of ascending node of orbital plane at weekly epoch [pi]
    i_0 = 0.0;				//!< Inclination angle at reference time [pi]
    Omega_dot = 0.0;		//!< Rate of right ascension [pi/s]
    i_0_dot = 0.0;			//!< Rate of inclination angle [pi/s]
    C_is = 0.0;				//!< Amplitude of sine harmonic correction term to the angle of inclination [rad]
    C_ic = 0.0;				//!< Amplitude of cosine harmonic correction term to the angle of inclination [rad]
    C_rs = 0.0;				//!< Amplitude of sine harmonic correction term to the orbit radius [m]
    C_rc = 0.0;				//!< Amplitude of cosine harmonic correction term to the orbit radius [m]
    C_us = 0.0;				//!< Amplitude of sine harmonic correction to the argument of latitude [rad]
    C_uc = 0.0;				//!< Amplitude of cosine harmonic correction to the argument of latitude [rad]


}


boost::posix_time::ptime Beidou_Cnav2_Ephemeris::compute_beidou_time(const double offset_time) const
{
	// Requires Propagation?
	/*
    boost::posix_time::time_duration t(0, 0, offset_time + d_tau_c + d_tau_n);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime beidou_time(d1 + d2, t);

    return beidou_time;
    */
}


boost::posix_time::ptime Beidou_Cnav2_Ephemeris::beidt_to_utc(const double offset_time, const double glot2utc_corr) const
{
    double tod = 0.0;
    double glot2utc = 3 * 3600;
    // Requires Propagation?
    	/*
    tod = offset_time - glot2utc + glot2utc_corr + d_tau_n;
    boost::posix_time::time_duration t(0, 0, tod);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime utc_time(d1 + d2, t);

    return utc_time;
    */
}


void Beidou_Cnav2_Ephemeris::glot_to_gpst(double tod_offset, double glot2utc_corr, double glot2gpst_corr, double* wn, double* tow) const
{
    double tod = 0.0;
    double beidt2utc = 3 * 3600;
    double days = 0.0;
    double total_sec = 0.0, sec_of_day = 0.0;
    int i = 0;
    // Requires Propagation?
    	/*
    boost::gregorian::date gps_epoch{1980, 1, 6};

    // tk is relative to UTC(SU) + 3.00 hrs, so we need to convert to utc and add corrections
    // tk plus 10 sec is the true tod since get_TOW is called when in str5
    tod = tod_offset - beidt2utc;

    boost::posix_time::time_duration t(0, 0, tod);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime utc_time(d1 + d2, t);
    boost::gregorian::date utc_date = utc_time.date();
    boost::posix_time::ptime gps_time;

    // Adjust for leap second correction
    for (i = 0; BEIDOU_LEAP_SECONDS[i][0] > 0; i++)
        {
            boost::posix_time::time_duration t3(BEIDOU_LEAP_SECONDS[i][3], BEIDOU_LEAP_SECONDS[i][4], BEIDOU_LEAP_SECONDS[i][5]);
            boost::gregorian::date d3(BEIDOU_LEAP_SECONDS[i][0], BEIDOU_LEAP_SECONDS[i][1], BEIDOU_LEAP_SECONDS[i][2]);
            boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We add the leap second when going from utc to gpst
                    gps_time = utc_time + boost::posix_time::time_duration(0, 0, fabs(BEIDOU_LEAP_SECONDS[i][6]));
                    break;
                }
        }

    // Total number of days
    std::string fdat = boost::posix_time::to_simple_string(gps_time);
    days = static_cast<double>((utc_date - gps_epoch).days());

    // Total number of seconds
    sec_of_day = static_cast<double>((gps_time.time_of_day()).total_seconds());
    total_sec = days * 86400 + sec_of_day;

    // Compute Week number
    *wn = floor(total_sec / 604800);

    // Compute the arithmetic modules to wrap around range
    *tow = total_sec - 604800 * floor(total_sec / 604800);
    // Perform corrections from fractional seconds
    *tow += glot2utc_corr + glot2gpst_corr;

    */
}

//  Add later
/*
double Beidou_Cnav2_Ephemeris::check_t(double time)
{
    double corrTime;
    double half_day = 43200.0;  // seconds
    corrTime = time;
    if (time > half_day)
        {
            corrTime = time - 2.0 * half_day;
        }
    else if (time < -half_day)
        {
            corrTime = time + 2.0 * half_day;
        }
    return corrTime;
}
*/

// FIXME Fix reference here
// 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
double Beidou_Cnav2_Ephemeris::sv_clock_drift(double transmitTime, double timeCorrUTC)
{
	// Requires Propagation?
		/*
    double dt;
    dt = check_t(transmitTime - d_t_b);
    d_satClkDrift = -(d_tau_n + timeCorrUTC - d_gamma_n * dt);
    //Correct satellite group delay and missing relativistic term here
    //d_satClkDrift-=d_TGD;

    return d_satClkDrift;
    */
}
