/*!
 * \file beidou_cnav2_ephemeris.h
 * \brief  Interface of a BEIDOU EPHEMERIS storage
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_EPHEMERIS_H_
#define GNSS_SDR_BEIDOU_CNAV2_EPHEMERIS_H_


#include <boost/serialization/nvp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


/*!
 * \brief This class is a storage and orbital model functions for the BEIDOU SV ephemeris data as described in beidou ICD (Edition 5.1)
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
 */
class Beidou_Cnav2_Ephemeris
{

public:

    double t_oe;			//!< Ephemeris reference time [s]
    double SatType;			//!< Satellite orbit type [dimensionless]
    double d_A;				//!< Semi-major axis difference at reference time [m]
    double A_dot;			//!< Change rate in semi-major axis [m/s]
    double dn_0;			//!< Mean motion difference from computed value at reference time [pi/s]
    double dn_0_dot;		//!< Rate of mean motion difference from computed value at reference time [pi/s^2]
    double M_0;				//!< Mean anomaly at reference time [pi]
    double e;				//!< Eccentricity [dimensionless]
    double omega;			//!< Argument of perigee [pi]
    double Omega_0;			//!< Longitude of ascending node of orbital plane at weekly epoch [pi]
    double i_0;				//!< Inclination angle at reference time [pi]
    double Omega_dot;		//!< Rate of right ascension [pi/s]
    double i_0_dot;			//!< Rate of inclination angle [pi/s]
    double C_is;			//!< Amplitude of sine harmonic correction term to the angle of inclination [rad]
    double C_ic;			//!< Amplitude of cosine harmonic correction term to the angle of inclination [rad]
    double C_rs;			//!< Amplitude of sine harmonic correction term to the orbit radius [m]
    double C_rc;			//!< Amplitude of cosine harmonic correction term to the orbit radius [m]
    double C_us;			//!< Amplitude of sine harmonic correction to the argument of latitude [rad]
    double C_uc;			//!< Amplitude of cosine harmonic correction to the argument of latitude [rad]

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };

        archive& make_nvp("t_oe", t_oe);  //!< SV PRN frequency channel number
        archive& make_nvp("SatType", SatType);
        archive& make_nvp("d_A", d_A);
        archive& make_nvp("A_dot", A_dot);
        archive& make_nvp("dn_0", dn_0);
        archive& make_nvp("dn_0_dot", dn_0_dot);
        archive& make_nvp("M_0", M_0);
        archive& make_nvp("e", e);
        archive& make_nvp("omega", omega);
        archive& make_nvp("Omega_0", Omega_0);
        archive& make_nvp("i_0", i_0);
        archive& make_nvp("Omega_dot", Omega_dot);
        archive& make_nvp("i_0_dot", i_0_dot);
        archive& make_nvp("C_is", C_is);
        archive& make_nvp("C_ic", C_ic);
        archive& make_nvp("C_rs", C_rs);
        archive& make_nvp("C_rc", C_rc);
        archive& make_nvp("C_us", C_us);
        archive& make_nvp("C_uc", C_uc);
    }

    /*!
     * \brief Sets (\a d_satClkDrift)and returns the clock drift in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_drift(double transmitTime, double timeCorrUTC);

    /*!
     *  \brief Computes the beidou System Time and returns a boost::posix_time::ptime object
     * \ param offset_time Is the start of day offset to compute the time
     */
    boost::posix_time::ptime compute_beidou_time(const double offset_time) const;

    /*!
     * \brief Converts from beidouT to UTC
     * \details The function simply adjust for the 6 hrs offset between beidouT and UTC
     * \param[in] offset_time Is the start of day offset
     * \param[in] glot2utc_corr Correction from beidouT to UTC
     * \returns UTC time as a boost::posix_time::ptime object
     */
    boost::posix_time::ptime beidt_to_utc(const double offset_time, const double glot2utc_corr) const;

    /*!
     * \brief Converts from beidouT to GPST
     * \details Converts from beidouT to GPST in time of week (TOW) and week number (WN) format
     * \param[in] tod_offset Is the start of day offset
     * \param[in] glot2utc_corr Correction from beidouT to UTC
     * \param[in] glot2gpst_corr Correction from beidouT to GPST
     * \param[out] WN Week Number, not in mod(1024) format
     * \param[out] TOW Time of Week in seconds of week
     */
    void glot_to_gpst(double tod_offset, double glot2utc_corr, double glot2gpst_corr, double* WN, double* TOW) const;

    /*!
     * Default constructor
     */
    Beidou_Cnav2_Ephemeris();
};

#endif
