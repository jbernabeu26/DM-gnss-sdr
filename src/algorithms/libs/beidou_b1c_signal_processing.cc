/*!
 * \file beidou_b1c_signal._processing.cc
 * \brief This class implements signal generators for the BEIDOU B1c signals
 * \author Andrew Kamble, 2019. andrewkamble88@gmail.com 
 * \note Code added as part of GSoC 2019 program
 * Detailed description of the file here if needed.
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


#include "beidou_b1c_signal_processing.h"
#include "Beidou_B1C.h"
#include <cinttypes>
#include <complex>
#include <fstream>
#include<iostream>
#include<cmath>
#include <vector>
#include <algorithm>

// Make legendre sequence for Primary Data and Primary Pilot codes
bool make_leg_primary(int32_t k)
{
    bool squaremodp = false;
    int32_t z = 1;

    if (k == 0)
        {
            squaremodp = true;
        }
    else
        {
            while (z <= (BEIDOU_B1C_WEIL_N - 1) / 2)
                {
                    squaremodp = false;
                    if (((z * z) % BEIDOU_B1C_WEIL_N) == k)
                        {
                            // k is a square(mod p)
                            squaremodp = true;
                            break;
                        }
                    z = z + 1;
                }
        }

    return squaremodp;
}

// Make B1C Data Primary code
std::deque<bool> make_b1cd_primary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cd_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cd_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
        }

    return trunc_weil_seq;
}

void make_b1cd(gsl::span<int32_t> _dest, int32_t prn)
{
	int32_t phase_diff = BEIDOU_B1Cd_PHASE_DIFF[prn-1];
	int32_t truncation_point = BEIDOU_B1Cd_TRUNC_POINT[prn-1];

	// Generate Data Primary code
	std::deque<bool> b1cd_primary_code =  make_b1cd_primary_weil_seq(phase_diff, truncation_point);

	for (uint32_t n = 0; n < 10230; n++)
	{
		_dest[n] = b1cd_primary_code[n];
	}
}



//! Generate a float version of the B1c Data Primary code
void beidou_b1cd_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
	uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

	for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}

// Generate a complex version of the B1c Data Primary code
void beidou_b1cd_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    	uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

        for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(1.0 - 2.0 * _code[ii]), 0.0);
        }
}

/*
 *  Generates complex BEIDOU B1c Data Primary code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1cd_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cd_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cd_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cd code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }
}

//=========================================PRIMARY CODE GENERATION OF PILOT COMPONENT=========================================

// Make B1C Pilot Primary code
std::deque<bool> make_b1cp_primary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cp_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
        }

    return trunc_weil_seq;
}

//! Generate the B1C Pilot Primary code
void make_b1cp(gsl::span<int32_t> _dest, int32_t prn)
{
	int32_t phase_diff = BEIDOU_B1Cp_PHASE_DIFF[prn-1];
	int32_t truncation_point = BEIDOU_B1Cp_TRUNC_POINT[prn-1];

	// Generate Data Primary code
	std::deque<bool> b1cp_primary_code =  make_b1cp_primary_weil_seq(phase_diff, truncation_point);

	for (uint32_t n = 0; n < 10230; n++)
	{
		_dest[n] = b1cp_primary_code[n];
	}
}


//! Generate a float version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
	uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

	for (uint32_t ii = 0; ii < _code_length; ii++)
        {
             _dest[ii] = 1.0 - 2.0 * static_cast<float>(_code[ii]);
        }
}

//! Generate a complex version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    	uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

        for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(0.0,1.0 - 2.0 * _code[ii]));
        }
}


/*
 *  Generates complex BEIDOU B1C Primary pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1cp_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1C code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}

//=========================================SECONDARY CODE GENERATION OF PILOT COMPONENT=========================================

bool make_leg_secondary(int32_t k)
{
    bool squaremodp = false;
    int32_t z = 1;

    if (k == 0)
        {
            squaremodp = true;
        }
    else
        {
            while (z <= (BEIDOU_B1C_WEIL_N_SECONDARY - 1) / 2)
                {
                    squaremodp = false;
                    if (((z * z) % BEIDOU_B1C_WEIL_N_SECONDARY) == k)
                        {
                            // k is a square(mod p)
                            squaremodp = true;
                            break;
                        }
                    z = z + 1;
                }
        }

    return squaremodp;
}

// Make B1C Pilot Secondary code
std::deque<bool> make_b1cp_secondary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N_SECONDARY;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_secondary(k) xor make_leg_secondary((k + w) % N);
        }

    return trunc_weil_seq;
}

//! Generate a version of the B1C Pilot code with the secondary pilot code included
void make_b1cp_secondary(gsl::span<int32_t> _dest, int32_t prn)
{
    
    int32_t phase_diff_primary = BEIDOU_B1Cp_PHASE_DIFF[prn-1];
    int32_t truncation_point_primary = BEIDOU_B1Cp_TRUNC_POINT[prn-1];
    
    int32_t phase_diff_secondary = BEIDOU_B1Cp_SECONDARY_PHASE_DIFF[prn - 1];
    int32_t truncation_point_secondary = BEIDOU_B1Cp_SECONDARY_TRUNC_POINT[prn - 1];
    
    // Generate primary code
    std::deque<bool> b1cp_primary_code = make_b1cp_primary_weil_seq(phase_diff_primary, truncation_point_primary);
    
    // Generate secondary code
    std::deque<bool> b1cp_sec_code = make_b1cp_secondary_weil_seq(phase_diff_secondary, truncation_point_secondary);

    for (uint32_t m = 0; m < BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; m++)
        {
            for (uint32_t n = 0; n < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; n++)
                {
                    _dest[n + m * BEIDOU_B1Cp_CODE_LENGTH_CHIPS] = b1cp_primary_code[n] xor b1cp_sec_code[m];
                }
        }
}


// Generate a complex version of the B1C pilot code with the secondary pilot code
void beidou_b1cp_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
        }
}


/*
 * Generates a float version of the B1C pilot primary code
 */
void beidou_b1cp_code_gen_float_secondary(gsl::span<float> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}


/*
 *  Generates complex BEIDOU B1C pilot code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
 */
void beidou_b1cp_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cp code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}



/*
 *  Generates complex BEIDOU B1C data+pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1c_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length_data = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    uint32_t _code_length_pilot = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    int32_t _code_pilot[_code_length_pilot];
    int32_t _code_data[_code_length_data];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code_pilot, _code_length_pilot), _prn);
            make_b1cd(gsl::span<int32_t>(_code_data, _code_length_data), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1C code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeLength - 1], 1.0 - 2.0 * _code_pilot[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeValueIndex], 1.0 - 2.0 * _code_pilot[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}

