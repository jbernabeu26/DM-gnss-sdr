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



//! Generation of the Legendre sequence for Primary code of Data and Pilot Components
std::vector < int32_t > generate_legendre_sequence()
{
	std::vector <int32_t> legendre(BEIDOU_B1C_WEIL_N);
	for(int i=0;i<BEIDOU_B1C_WEIL_N;i++)
	{
		legendre[i]=0;
	}
	for(int k=1;k<BEIDOU_B1C_WEIL_N;k++)
	{
		int leg = 0;
		for(int x=1;x<BEIDOU_B1C_WEIL_N;x++)
		{
			int power= pow(x,2);
			if(( power % BEIDOU_B1C_WEIL_N) == k)
			{	
				leg = 1;
				break;
			}
		}
		legendre[k] = leg;
	}
	return legendre;
}
//! Generation of the Legendre sequence for Secondary code of Pilot Components
std::vector < int32_t > generate_legendre_sequence_secondary()
{
	std::vector<int32_t>legendre(BEIDOU_B1C_WEIL_N_SECONDARY);
	for(int i=0;i<BEIDOU_B1C_WEIL_N_SECONDARY;i++)
	{
		legendre[i]=0;
	}
	for(int k=1;k<BEIDOU_B1C_WEIL_N_SECONDARY;k++)
	{
		int leg = 0;
		for(int x=1;x<BEIDOU_B1C_WEIL_N_SECONDARY;x++)
		{
			int power= pow(x,2);
			if(( power % BEIDOU_B1C_WEIL_N_SECONDARY) == k)
			{	
				leg = 1;
				break;
			}
		}
		legendre[k] = leg;
	}
	return legendre;
}

//! Generate the B1c Data Primary codes
void make_b1cd(int32_t* _dest, int prn)
{	
	std::vector <int32_t> legendre(BEIDOU_B1C_WEIL_N);
	legendre.clear() ;
	legendre = generate_legendre_sequence();
	//creating new vector for rotation	
	std::vector<int32_t> legendre1(BEIDOU_B1C_WEIL_N);
	//copying the elements of legendre in legendre1 for computing weil 
	legendre1 = legendre;
	
	int32_t rotL1=BEIDOU_B1Cd_PHASE_DIFF[prn-1];
	std::rotate(legendre1.begin(),legendre1.begin()+rotL1,legendre1.end());
	
	int32_t weil[BEIDOU_B1C_WEIL_N];
	for(int i=0;i<BEIDOU_B1C_WEIL_N;i++)
	{	
		weil[i]=legendre[i] xor legendre1[i];
	}
	//copying the elements of weil in weil1 for computing weil_rot
	std::vector <int32_t> weil1(weil,weil+(BEIDOU_B1C_WEIL_N));
	int32_t rotL2= BEIDOU_B1Cd_TRUNC_POINT[prn-1]- 1;
	std::rotate(weil1.begin(),weil1.begin()+rotL2,weil1.end());
	
	int32_t weil_rot[BEIDOU_B1C_WEIL_N];
	for (int i=0; i < BEIDOU_B1C_WEIL_N; i++)
	{ 
		weil_rot[i]=weil1[i];
	}
	//Truncating weil and taking only 10230 chips
	for(int i=0;i<BEIDOU_B1Cd_CODE_LENGTH_CHIPS;i++)
	{
		_dest[i]=weil_rot[i];
	}
}

//! Generate a float version of the B1c Data Primary code
void beidou_b1cd_code_gen_float(float* _dest, unsigned int _prn)
{
    	int32_t* _code = new int32_t[BEIDOU_B1Cd_CODE_LENGTH_CHIPS];

    	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(_code, _prn );
        }

    	for (signed int i = 0; i < BEIDOU_B1Cd_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    	delete[] _code;
}

// Generate a complex version of the B1c Data Primary code
void beidou_b1cd_code_gen_complex(std::complex<float>* _dest, unsigned int _prn)
{
    	int32_t* _code = new int32_t[BEIDOU_B1Cd_CODE_LENGTH_CHIPS];

    	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(_code, _prn );
        }

    	for (signed int i = 0; i < BEIDOU_B1Cd_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    	delete[] _code;
}


/*
 *  Generates complex BEIDOU B1c Data Primary code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1cd_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cd_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn <= 63)
        {
            make_b1cd(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cd_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cd_CODE_RATE_HZ);  // code chip period in sec

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1c code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1cd code -----------------------
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
    delete[] _code;
}

//=========================================PRIMARY CODE GENERATION OF PILOT COMPONENT=========================================

//! Generate the B1c Pilot Primary code
void make_b1cp(int32_t* _dest, int prn)
{	
	std::vector <int32_t> legendre(BEIDOU_B1C_WEIL_N);
	legendre.clear() ;
	legendre = generate_legendre_sequence();
	//creating new array for rotation	
	std::vector<int32_t> legendre1(BEIDOU_B1C_WEIL_N);
	//copying the elements of legendre in legendre1 for computing weil & copying in new vector for rotation
	legendre1 = legendre;
	
	int32_t rotL1=BEIDOU_B1Cp_PHASE_DIFF[prn-1];
	std::rotate(legendre1.begin(),legendre1.begin()+rotL1,legendre1.end());
	
	int32_t weil[BEIDOU_B1C_WEIL_N];
	for(int i=0;i<BEIDOU_B1C_WEIL_N;i++)
	{	
		weil[i]=legendre[i] xor legendre1[i];
	}
	//copying the elements of weil in weil1 for computing weil_rot
	std::vector <int32_t> weil1(weil,weil+(BEIDOU_B1C_WEIL_N));
	int32_t rotL2= BEIDOU_B1Cp_TRUNC_POINT[prn-1]- 1;
	std::rotate(weil1.begin(),weil1.begin()+rotL2,weil1.end());
	
	int32_t weil_rot[BEIDOU_B1C_WEIL_N];
	for (int i=0; i < BEIDOU_B1C_WEIL_N; i++)
	{ 
		weil_rot[i]=weil1[i];
	}
	//Truncating weil and taking only 10230
	for(int i=0;i<BEIDOU_B1Cp_CODE_LENGTH_CHIPS;i++)
	{
		_dest[i]=weil_rot[i];
	}
}


//! Generates a float version of the B1c Pilot Primary code
void beidou_b1cp_code_gen_float(float* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cp_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(_code, _prn);
        }

    for (signed int i = 0; i < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    delete[] _code;
}

//! Generates a complex version of the B1c Pilot Primary code
void beidou_b1cp_code_gen_complex(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cp_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(_code, _prn);
        }

    for (signed int i = 0; i < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}


//! Generates complex BEIDOU B1c Pilot Primary code for the desired SV ID and sampled to specific sampling frequency
void beidou_b1cp_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cp_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1c pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1c code -----------------------
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
    delete[] _code;
}

//=========================================SECONDARY CODE GENERATION OF PILOT COMPONENT=========================================

//! Generate a version of the B1c Pilot Secondary code
void make_b1cp_secondary(int32_t* _dest, int prn)
{
	std::vector<int32_t>legendre(BEIDOU_B1C_WEIL_N_SECONDARY);
	legendre.clear() ;
	legendre = generate_legendre_sequence_secondary();
	//creating new vector for rotation	
	std::vector<int32_t> legendre1(BEIDOU_B1C_WEIL_N_SECONDARY);
	//copying the elements of legendre in legendre1 for computing weil 
	legendre1 = legendre;
	
	int32_t rotL1=BEIDOU_B1Cp_SECONDARY_PHASE_DIFF[prn-1];
	std::rotate(legendre1.begin(),legendre1.begin()+rotL1,legendre1.end());
	
	int32_t weil[BEIDOU_B1C_WEIL_N_SECONDARY];
	for(int i=0;i<BEIDOU_B1C_WEIL_N_SECONDARY;i++)
	{	
		weil[i]=legendre[i] xor legendre1[i];
	}
	//copying the elements of weil in weil1 for computing weil_rot
	std::vector<int32_t> weil1(weil,weil+(BEIDOU_B1C_WEIL_N_SECONDARY));
	int32_t rotL2= BEIDOU_B1Cp_SECONDARY_TRUNC_POINT[prn-1]- 1;
	std::rotate(weil1.begin(),weil1.begin()+rotL2,weil1.end());
	
	int32_t weil_rot[BEIDOU_B1C_WEIL_N_SECONDARY];
	for (int i=0; i < BEIDOU_B1C_WEIL_N_SECONDARY; i++)
	{ 
		weil_rot[i]=weil1[i];
	}
	//Truncating weil and taking only 1800 chips
	for(int i=0;i<BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;i++)
	{
		_dest[i]=weil_rot[i];
	}
}

//! Generate a complex version of the B1c Pilot Secondary code
void beidou_b1cp_code_gen_complex_secondary(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cp_CODE_LENGTH_CHIPS*BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp_secondary(_code, _prn );
        }

    for (signed int i = 0; i <BEIDOU_B1Cp_CODE_LENGTH_CHIPS*BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}

//! Generates complex BEIDOU B1c Pilot Secondary code for the desired SV ID and sampled to specific sampling frequency
void beidou_b1cp_code_gen_complex_sampled_secondary(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B1Cp_CODE_LENGTH_CHIPS*BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn <= 63)
        {
            make_b1cp_secondary(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS*BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // code chip period in sec

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1c code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1cp code -----------------------
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
    delete[] _code;
}
