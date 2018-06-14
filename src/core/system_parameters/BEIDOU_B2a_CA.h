/*!
 * \file BEIDOU_B2a_CA.h
 * \brief  Defines system parameters for Beidou B2a C/A signal and NAV data
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
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


#ifndef GNSS_SDR_BEIDOU_B2a_CA_H_
#define GNSS_SDR_BEIDOU_B2a_CA_H_

#include "gnss_frequencies.h"
#include "MATH_CONSTANTS.h"
#include <map>
#include <vector>
#include <utility>  // std::pair


// Physical constants
const double BEIDOU_C_m_s = SPEED_OF_LIGHT;                   //!< The speed of light, [m/s]
const double BEIDOU_C_m_ms = 299792.4580;                     //!< The speed of light, [m/ms]
const double BEIDOU_PI = 3.1415926535898;                     //!< Pi as defined in IS-GPS-200E
const double BEIDOU_TWO_PI = 6.283185307179586;               //!< 2Pi as defined in IS-GPS-200E
const double BEIDOU_OMEGA_EARTH_DOT = 7.292115e-5;            //!< Earth rotation rate, [rad/s]
const double BEIDOU_GM = 398600.4418e9;                       //!< Geocentric gravitational constant, [m^3/s^2]
const double BEIDOU_fM_a = 0.35e9;                            //!< Gravitational constant of atmosphere [m^3/s^2]
const double BEIDOU_SEMI_MAJOR_AXIS = 6378137;                //!< Semi-major axis of Earth [m]
const double BEIDOU_FLATTENING = 1 / 298.257222101;                //!< Flattening parameter
const double BEIDOU_GRAVITY = 97803284;                       //!< Equatorial acceleration of gravity [mGal]
const double BEIDOU_GRAVITY_CORRECTION = 0.87;                //!< Correction to acceleration of gravity at sea-level due to Atmosphere[мGal]
const double BEIDOU_J2 = 1082625.75e-9;                       //!< Second zonal harmonic of the geopotential
const double BEIDOU_J4 = -2370.89e-9;                         //!< Fourth zonal harmonic of the geopotential
const double BEIDOU_J6 = 6.08e-9;                             //!< Sixth zonal harmonic of the geopotential
const double BEIDOU_J8 = 1.40e-11;                            //!< Eighth zonal harmonic of the geopotential
const double BEIDOU_U0 = 62636861.4;                          //!< Normal potential at surface of common terrestrial ellipsoid [m^2/s^2]
const double BEIDOU_C20 = -1082.63e-6;                        //!< Second zonal coefficient of spherical harmonic expansion
const double BEIDOU_EARTH_RADIUS = 6378.136;                  //!< Equatorial radius of Earth [km]
const double BEIDOU_EARTH_INCLINATION = 0.000409148809899e3;  //!< Mean inclination of ecliptic to equator (23 deg 26 min 33 sec) [rad]

const double BEIDOU_TAU_0 = -0.005835151531174e3;  //!< (-334 deg 19 min 46.40 sec) [rad];
const double BEIDOU_TAU_1 = 0.071018041257371e3;   //!< (4069 deg 02 min 02.52 sec) [rad];

const double BEIDOU_MOON_Q0 = -0.001115184961435e3;          //!< (-63 deg 53 min 43.41 sec) [rad]
const double BEIDOU_MOON_Q1 = 8.328691103668023e3;           //!< (477198 deg 50 min 56.79 sec) [rad]
const double BEIDOU_MOON_OMEGA_0 = 0.004523601514852e3;      //!< (259 deg 10 min 59.79 sec) [rad]
const double BEIDOU_MOON_OMEGA_1 = -0.033757146246552e3;     //!< (-1934 deg 08 min 31.23 sec) [rad]
const double BEIDOU_MOON_GM = 4902.835;                      //!< Lunar gravitational constant [km^3/s^2]
const double BEIDOU_MOON_SEMI_MAJOR_AXIS = 3.84385243e5;     //!< Semi-major axis of lunar orbit [km];
const double BEIDOU_MOON_ECCENTRICITY = 0.054900489;         //!< Eccentricity of lunar orbit
const double BEIDOU_MOON_INCLINATION = 0.000089803977407e3;  //!< Inclination of lunar orbit to ecliptic plane (5 deg 08 min 43.4 sec) [rad]

const double BEIDOU_SUN_OMEGA = 0.004908229466869e3;  //!< TODO What is this operation in the seconds with T?(281 deg 13 min 15.0 + 6189.03*Т sec) [rad]
const double BEIDOU_SUN_Q0 = 0.006256583774423e3;     //!< (358 deg 28 min 33.04 sec) [rad]
const double BEIDOU_SUN_Q1 = 0e3;                     //!< TODO Why is the value greater than 60?(129596579.10 sec) [rad]
const double BEIDOU_SUN_GM = 0.1325263e12;            //!< Solar gravitational constant [km^3/s^2]
const double BEIDOU_SUN_SEMI_MAJOR_AXIS = 1.49598e8;  //!< Semi-major axis of solar orbit [km];
const double BEIDOU_SUN_ECCENTRICITY = 0.016719;      //!< Eccentricity of solar orbit

const double BEIDOU_B2a_CA_FREQ_HZ = FREQ2_BDS;        //!< BEIDOU B2a carrier frequency [Hz]
const double BEIDOU_B2a_CA_CODE_RATE_HZ = 0.511e6;     //!< BEIDOU L1 C/A code rate [chips/s]
const double BEIDOU_B2a_CA_CODE_LENGTH_CHIPS = 10230;  //!< BEIDOU B2a C/A code length [chips]
const double BEIDOU_B2a_CA_CODE_PERIOD = 0.001;        //!< BEIDOU B2a C/A code period [seconds]
const double BEIDOU_B2a_CA_CHIP_PERIOD = 1.9569e-06;   //!< BEIDOU L1 C/A chip period [seconds]
const double BEIDOU_B2a_CA_SYMBOL_RATE_SPS = 200;
const double BEIDOU_B2a_CA_SYMBOL_Length = 600;

const int BEIDOU_CA_NBR_SATS = 24;  // STRING DATA WITHOUT PREAMBLE

/*!
 * \brief Record of leap seconds definition for GLOT to GPST conversion and vice versa
 * \details Each entry is defined by an array of 7 elements consisting of yr,month,day,hr,min,sec,utc-gpst
 * \note Ideally should use leap seconds definitions of rtklib
 */
const double BEIDOU_LEAP_SECONDS[19][7] = {
    {2017, 1, 1, 0, 0, 0, -18},
    {2015, 7, 1, 0, 0, 0, -17},
    {2012, 7, 1, 0, 0, 0, -16},
    {2009, 1, 1, 0, 0, 0, -15},
    {2006, 1, 1, 0, 0, 0, -14},
    {1999, 1, 1, 0, 0, 0, -13},
    {1997, 7, 1, 0, 0, 0, -12},
    {1996, 1, 1, 0, 0, 0, -11},
    {1994, 7, 1, 0, 0, 0, -10},
    {1993, 7, 1, 0, 0, 0, -9},
    {1992, 7, 1, 0, 0, 0, -8},
    {1991, 1, 1, 0, 0, 0, -7},
    {1990, 1, 1, 0, 0, 0, -6},
    {1988, 1, 1, 0, 0, 0, -5},
    {1985, 7, 1, 0, 0, 0, -4},
    {1983, 7, 1, 0, 0, 0, -3},
    {1982, 7, 1, 0, 0, 0, -2},
    {1981, 7, 1, 0, 0, 0, -1},
    {}};

//!< BEIDOU SV's orbital slots PRN = (orbital_slot - 1)
const std::map<unsigned int, int> BEIDOU_PRN = {
    {
        0,
        8,
    },  //For test
    {
        1,
        1,
    },  //Plane 1
    {
        2,
        -4,
    },  //Plane 1
    {
        3,
        5,
    },  //Plane 1
    {
        4,
        6,
    },  //Plane 1
    {
        5,
        1,
    },  //Plane 1
    {
        6,
        -4,
    },  //Plane 1
    {
        7,
        5,
    },  //Plane 1
    {
        8,
        6,
    },  //Plane 1
    {
        9,
        -2,
    },  //Plane 2
    {
        10,
        -7,
    },  //Plane 2
    {
        11,
        0,
    },  //Plane 2
    {
        12,
        -1,
    },  //Plane 2
    {
        13,
        -2,
    },  //Plane 2
    {
        14,
        -7,
    },  //Plane 2
    {
        15,
        0,
    },  //Plane 2
    {
        16,
        -1,
    },  //Plane 2
    {
        17,
        4,
    },  //Plane 3
    {
        18,
        -3,
    },  //Plane 3
    {
        19,
        3,
    },  //Plane 3
    {
        20,
        -5,
    },  //Plane 3
    {
        21,
        4,
    },  //Plane 3
    {
        22,
        -3,
    },  //Plane 3
    {
        23,
        3,
    },         //Plane 3
    {24, 2}};  //Plane 3

const double BEIDOU_STARTOFFSET_ms = 68.802;  //[ms] Initial sign. travel time (this cannot go here)

// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
const int BEIDOU_L1_CA_HISTORY_DEEP = 100;

// NAVIGATION MESSAGE DEMODULATION AND DECODING
#define BEIDOU_CNAV2_PREAMBLE                                                                    \
    {                                                                                            \
        1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0 \
    }
const double BEIDOU_CNAV2_PREAMBLE_DURATION_S = 0.300;
const int BEIDOU_CNAV2_PREAMBLE_LENGTH_BITS = 30;
const int BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS = 300;
const int BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS = 2000;
const int BEIDOU_CNAV2_TELEMETRY_RATE_BITS_SECOND = 50;  //!< NAV message bit rate [bits/s]
const int BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_BIT = 10;
const int BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT = 10;
const int BEIDOU_CNAV2_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_CNAV2_TELEMETRY_RATE_BITS_SECOND * BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_BIT;  //!< NAV message bit rate [symbols/s]
const int BEIDOU_CNAV2_STRING_SYMBOLS = 2000;                                                                                             //!< Number of bits per string in the CNAV2 message (85 data bits + 30 time mark bits) [bits]
const int BEIDOU_CNAV2_STRING_BITS = 85;                                                                                                  //!< Number of bits per string in the CNAV2 message (85 data bits + 30 time mark bits) [bits]
const int BEIDOU_CNAV2_HAMMING_CODE_BITS = 8;                                                                                             //!< Number of bits in hamming code sequence of CNAV2 message
const int BEIDOU_CNAV2_DATA_SYMBOLS = 1700;                                                                                               // STRING DATA WITHOUT PREAMBLE

const std::vector<int> BEIDOU_CNAV2_CRC_I_INDEX{9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84};
const std::vector<int> BEIDOU_CNAV2_CRC_J_INDEX{9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45, 48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84};
const std::vector<int> BEIDOU_CNAV2_CRC_K_INDEX{10, 11, 12, 16, 17, 18, 19, 23, 24, 25, 26, 31, 32, 33, 34, 38, 39, 40, 41, 46, 47, 48, 49, 54, 55, 56, 57, 62, 63, 64, 65, 69, 70, 71, 72, 77, 78, 79, 80, 85};
const std::vector<int> BEIDOU_CNAV2_CRC_L_INDEX{13, 14, 15, 16, 17, 18, 19, 27, 28, 29, 30, 31, 32, 33, 34, 42, 43, 44, 45, 46, 47, 48, 49, 58, 59, 60, 61, 62, 63, 64, 65, 73, 74, 75, 76, 77, 78, 79, 80};
const std::vector<int> BEIDOU_CNAV2_CRC_M_INDEX{20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 81, 82, 83, 84, 85};
const std::vector<int> BEIDOU_CNAV2_CRC_N_INDEX{35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65};
const std::vector<int> BEIDOU_CNAV2_CRC_P_INDEX{66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85};
const std::vector<int> BEIDOU_CNAV2_CRC_Q_INDEX{9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85};

// BEIDOU CNAV2 NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS

// Types

// Common
const std::vector<std::pair<int, int>> PRN({{1, 6}});
const std::vector<std::pair<int, int>> MesType({{7, 6}});
const std::vector<std::pair<int, int>> SOW({{13, 18}});

// Type 10 (288 bits)
const std::vector<std::pair<int, int>> WN({{31, 13}});
const std::vector<std::pair<int, int>> DIF({{44, 1}});
const std::vector<std::pair<int, int>> SIF({{45, 1}});
const std::vector<std::pair<int, int>> AIF({{46, 1}});
const std::vector<std::pair<int, int>> SISMAI({{47, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{51, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{52, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{53, 1}});
const std::vector<std::pair<int, int>> IODE({{54, 8}});
const std::vector<std::pair<int, int>> EPHEMERIS_1({{62, 203}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 11 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> EPHEMERIS_2({{43, 222}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 30 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> Clock_Correction_Parameters({{43, 69}});
const std::vector<std::pair<int, int>> IODC({{112, 10}});
const std::vector<std::pair<int, int>> T_GDB2ap({{122, 12}});
const std::vector<std::pair<int, int>> ISC_B2ad({{134, 12}});
const std::vector<std::pair<int, int>> Ionospheric_Delay_Correction_Model_Parameters({{146, 74}});
const std::vector<std::pair<int, int>> T_GDB1Cp({{220, 12}});
const std::vector<std::pair<int, int>> Rev({{232, 33}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 31 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> Clock_Correction_Parameters({{43, 69}});
const std::vector<std::pair<int, int>> IODC({{112, 10}});
const std::vector<std::pair<int, int>> WN_a({{122, 13}});
const std::vector<std::pair<int, int>> t_oa({{135, 8}});
const std::vector<std::pair<int, int>> Reduced_Almanac({{143, 38}});
const std::vector<std::pair<int, int>> Reduced_Almanac({{181, 38}});
const std::vector<std::pair<int, int>> Reduced_Alamanc({{219, 38}});
const std::vector<std::pair<int, int>> Rev({{257, 8}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 32 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> Clock_Correction_Parameters({{43, 69}});
const std::vector<std::pair<int, int>> IODC({{112, 10}});
const std::vector<std::pair<int, int>> EOP_Parameters({{122, 138}});
const std::vector<std::pair<int, int>> Rev({{260, 5}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 33 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> Clock_Correction_Parameters({{43, 69}});
const std::vector<std::pair<int, int>> BGTO_Parameters({{112, 68}});
const std::vector<std::pair<int, int>> Reduced_Almanac({{180, 38}});
const std::vector<std::pair<int, int>> IODC({{218, 10}});
const std::vector<std::pair<int, int>> WN_a({{228, 13}});
const std::vector<std::pair<int, int>> t_oa({{241, 8}});
const std::vector<std::pair<int, int>> Rev({{249, 16}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});


// Type 34 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> SISAI_OC({{43, 22}});
const std::vector<std::pair<int, int>> Clock_Correction_Parameters({{65, 69}});
const std::vector<std::pair<int, int>> IODC({{134, 10}});
const std::vector<std::pair<int, int>> BDT_UTC_Time_Offset_Parameters({{144, 97}});
const std::vector<std::pair<int, int>> Rev({{241, 24}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 40 (288 bits)
const std::vector<std::pair<int, int>> HS({{31, 2}});
const std::vector<std::pair<int, int>> DIF({{33, 1}});
const std::vector<std::pair<int, int>> SIF({{34, 1}});
const std::vector<std::pair<int, int>> AIF({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C({{42, 1}});
const std::vector<std::pair<int, int>> SISAI_OE({{43, 5}});
const std::vector<std::pair<int, int>> SISAI_OC({{48, 22}});
const std::vector<std::pair<int, int>> Midi_Almanac({{70, 156}});
const std::vector<std::pair<int, int>> Rev({{226, 39}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});


// Data Blocks This should be simply added to the data type above
// Ephemeris I (203 bits)
const std::vector<std::pair<int, int>> t_oe({{1, 11}});
const std::vector<std::pair<int, int>> SatType({{12, 2}});
const std::vector<std::pair<int, int>> dA({{14, 26}});
const std::vector<std::pair<int, int>> A_dot({{40, 25}});
const std::vector<std::pair<int, int>> dn_0({{65, 17}});
const std::vector<std::pair<int, int>> dn_0_dot({{82, 23}});
const std::vector<std::pair<int, int>> M_0({{105, 33}});
const std::vector<std::pair<int, int>> e({{138, 33}});
const std::vector<std::pair<int, int>> omega({{171, 33}});

// Ephemeris II (222 bits)
const std::vector<std::pair<int, int>> Omega_0({{1, 33}});
const std::vector<std::pair<int, int>> i_0({{34, 33}});
const std::vector<std::pair<int, int>> Omega_dot({{67, 19}});
const std::vector<std::pair<int, int>> i_0_dot({{86, 15}});
const std::vector<std::pair<int, int>> C_IS({{101, 16}});
const std::vector<std::pair<int, int>> C_IC({{117, 16}});
const std::vector<std::pair<int, int>> C_RS({{133, 24}});
const std::vector<std::pair<int, int>> C_RC({{157, 24}});
const std::vector<std::pair<int, int>> C_US({{181, 21}});
const std::vector<std::pair<int, int>> C_UC({{202, 21}});

// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc({{1, 11}});
const std::vector<std::pair<int, int>> a_0({{12, 25}});
const std::vector<std::pair<int, int>> a_1({{37, 22}});
const std::vector<std::pair<int, int>> a_2({{59, 11}});

// SISAI_OC (22 bits)
const std::vector<std::pair<int, int>> C_US({{1, 11}});
const std::vector<std::pair<int, int>> C_US({{12, 5}});
const std::vector<std::pair<int, int>> C_US({{17, 3}});
const std::vector<std::pair<int, int>> C_US({{20, 3}});

// Ionospheric Delay Correction Model Parameters (74 bits)
const std::vector<std::pair<int, int>> C_US({{1, 10}});
const std::vector<std::pair<int, int>> C_US({{11, 8}});
const std::vector<std::pair<int, int>> C_US({{19, 8}});
const std::vector<std::pair<int, int>> C_US({{27, 8}});
const std::vector<std::pair<int, int>> C_US({{35, 8}});
const std::vector<std::pair<int, int>> C_US({{43, 8}});
const std::vector<std::pair<int, int>> C_US({{51, 8}});
const std::vector<std::pair<int, int>> C_US({{59, 8}});
const std::vector<std::pair<int, int>> C_US({{67, 8}});

// BDT-UTC Time Offset Parameters (97 bits)
const std::vector<std::pair<int, int>> C_US({{1, 16}});
const std::vector<std::pair<int, int>> C_US({{17, 13}});
const std::vector<std::pair<int, int>> C_US({{30, 7}});
const std::vector<std::pair<int, int>> C_US({{37, 8}});
const std::vector<std::pair<int, int>> C_US({{45, 16}});
const std::vector<std::pair<int, int>> C_US({{61, 13}});
const std::vector<std::pair<int, int>> C_US({{74, 13}});
const std::vector<std::pair<int, int>> C_US({{87, 3}});
const std::vector<std::pair<int, int>> C_US({{90, 8}});

// Reduced Almanac Parameters (38 bits)
const std::vector<std::pair<int, int>> C_US({{1, 6}});
const std::vector<std::pair<int, int>> C_US({{7, 2}});
const std::vector<std::pair<int, int>> C_US({{9, 8}});
const std::vector<std::pair<int, int>> C_US({{17, 7}});
const std::vector<std::pair<int, int>> C_US({{24, 7}});
const std::vector<std::pair<int, int>> C_US({{31, 8}});

// EOP Parameters (138 bits)
const std::vector<std::pair<int, int>> C_US({{1, 16}});
const std::vector<std::pair<int, int>> C_US({{17, 21}});
const std::vector<std::pair<int, int>> C_US({{38, 15}});
const std::vector<std::pair<int, int>> C_US({{53, 21}});
const std::vector<std::pair<int, int>> C_US({{74, 15}});
const std::vector<std::pair<int, int>> C_US({{89, 31}});
const std::vector<std::pair<int, int>> C_US({{120, 19}});

// BGTO Parameters (68 bits)
const std::vector<std::pair<int, int>> C_US({{1, 3}});
const std::vector<std::pair<int, int>> C_US({{4, 13}});
const std::vector<std::pair<int, int>> C_US({{17, 16}});
const std::vector<std::pair<int, int>> C_US({{33, 16}});
const std::vector<std::pair<int, int>> C_US({{49, 13}});
const std::vector<std::pair<int, int>> C_US({{62, 7}});

// Midi Almanac Parameters (156 bits)
const std::vector<std::pair<int, int>> C_US({{1, 6}});
const std::vector<std::pair<int, int>> C_US({{7, 2}});
const std::vector<std::pair<int, int>> C_US({{9, 13}});
const std::vector<std::pair<int, int>> C_US({{22, 8}});
const std::vector<std::pair<int, int>> C_US({{30, 11}});
const std::vector<std::pair<int, int>> C_US({{41, 11}});
const std::vector<std::pair<int, int>> C_US({{52, 17}});
const std::vector<std::pair<int, int>> C_US({{69, 16}});
const std::vector<std::pair<int, int>> C_US({{85, 11}});
const std::vector<std::pair<int, int>> C_US({{96, 16}});
const std::vector<std::pair<int, int>> C_US({{112, 16}});
const std::vector<std::pair<int, int>> C_US({{128, 11}});
const std::vector<std::pair<int, int>> C_US({{139, 10}});
const std::vector<std::pair<int, int>> C_US({{149, 8}});

#endif /* GNSS_SDR_BEIDOU_L1_L2_CA_H_ */
