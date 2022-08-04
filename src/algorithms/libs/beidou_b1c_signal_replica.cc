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



#include "beidou_b1c_signal_replica.h"
#include "Beidou_B1C.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <complex>
#include <deque>
#include <fstream>
#include <iostream>
#include <vector>


const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>(x + 1)); };

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
   std::deque<bool> trunc_weil_seq(BEIDOU_B1C_CODE_LENGTH_CHIPS, 0);

   // Generate only the truncated sequence
   for (n = 0; n < BEIDOU_B1C_CODE_LENGTH_CHIPS; n++)
       {
           k = (n + p - 1) % N;
           trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
       }

   return trunc_weil_seq;
}

void make_b1cd(own::span<int32_t> _dest, int32_t prn)
{
   int32_t phase_diff = BEIDOU_B1Cd_PHASE_DIFF[prn - 1];
   int32_t truncation_point = BEIDOU_B1Cd_TRUNC_POINT[prn - 1];

   // Generate Data Primary code
   std::deque<bool> b1cd_primary_code = make_b1cd_primary_weil_seq(phase_diff, truncation_point);

   for (uint32_t n = 0; n < BEIDOU_B1C_CODE_LENGTH_CHIPS; n++)
       {
           _dest[n] = b1cd_primary_code[n];
       }
}


//! Generate a float version of the B1c Data Primary code
void beidou_b1cd_code_gen_float(own::span<float> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cd(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
       }
}

// Generate a complex version of the B1c Data Primary code
void beidou_b1cd_code_gen_complex(own::span<std::complex<float>> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cd(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = std::complex<float>(static_cast<float>(1.0 - 2.0 * _code[ii]), 0.0);
       }
}

/*
*  Generates complex BEIDOU B1c Data Primary code for the desired SV ID and sampled to specific sampling frequency
*/
void beidou_b1cd_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn <= 63)
       {
           make_b1cd(own::span<int32_t>(_code, _code_length), _prn);
       }

   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;

   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length)));

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // code chip period in sec

   for (int32_t i = 0; i < _samples_per_code; i++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C code values -------------------------
           _code_value_index = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1Cd code -----------------------
           if (i == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_code_length - 1], 0);
               }
           else
               {
                   _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_code_value_index], 0);  //repeat the chip -> upsample
               }
       }
}

//=========================================PRIMARY CODE GENERATION OF PILOT COMPONENT=========================================

// Make B1C Pilot Primary code
std::deque<bool> make_b1cp_primary_weil_seq(int32_t w, int32_t p)
{
   int32_t n, k = 0;
   int32_t N = BEIDOU_B1C_WEIL_N;
   std::deque<bool> trunc_weil_seq(BEIDOU_B1C_CODE_LENGTH_CHIPS, 0);

   // Generate only the truncated sequence
   for (n = 0; n < BEIDOU_B1C_CODE_LENGTH_CHIPS; n++)
       {
           k = (n + p - 1) % N;
           trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
       }

   return trunc_weil_seq;
}

//! Generate the B1C Pilot Primary code
void make_b1cp(own::span<int32_t> _dest, int32_t prn)
{
   int32_t phase_diff = BEIDOU_B1Cp_PHASE_DIFF[prn - 1];
   int32_t truncation_point = BEIDOU_B1Cp_TRUNC_POINT[prn - 1];

   // Generate Data Primary code
   std::deque<bool> b1cp_primary_code = make_b1cp_primary_weil_seq(phase_diff, truncation_point);

   for (uint32_t n = 0; n < 10230; n++)
       {
           _dest[n] = b1cp_primary_code[n];
       }
}


//! Generate a float version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_float(own::span<float> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = 1.0 - 2.0 * static_cast<float>(_code[ii]);
       }
}

//! Generate a complex version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_complex(own::span<std::complex<float>> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
       }
}


/*
*  Generates complex BEIDOU B1C Primary pilot code for the desired SV ID and sampled to specific sampling frequency
*/
void beidou_b1cp_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp(own::span<int32_t>(_code, _code_length), _prn);
       }

   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;

   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length)));

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // C/A chip period in sec

   //float aux;
   for (int32_t ii = 0; ii < _samples_per_code; ii++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C pilot code values -------------------------
           _code_value_index = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1C code -----------------------
           if (ii == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_code_length - 1]);
               }
           else
               {
                   _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_code_value_index]);  //repeat the chip -> upsample
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
void make_b1cp_secondary(own::span<int32_t> _dest, int32_t prn)
{
   int32_t phase_diff_primary = BEIDOU_B1Cp_PHASE_DIFF[prn - 1];
   int32_t truncation_point_primary = BEIDOU_B1Cp_TRUNC_POINT[prn - 1];

   int32_t phase_diff_secondary = BEIDOU_B1Cp_SECONDARY_PHASE_DIFF[prn - 1];
   int32_t truncation_point_secondary = BEIDOU_B1Cp_SECONDARY_TRUNC_POINT[prn - 1];

   // Generate primary code
   std::deque<bool> b1cp_primary_code = make_b1cp_primary_weil_seq(phase_diff_primary, truncation_point_primary);

   // Generate secondary code
   std::deque<bool> b1cp_sec_code = make_b1cp_secondary_weil_seq(phase_diff_secondary, truncation_point_secondary);

   for (uint32_t m = 0; m < BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; m++)
       {
           for (uint32_t n = 0; n < BEIDOU_B1C_CODE_LENGTH_CHIPS; n++)
               {
                   _dest[n + m * BEIDOU_B1C_CODE_LENGTH_CHIPS] = b1cp_primary_code[n] xor b1cp_sec_code[m];
               }
       }
}


// Generate a complex version of the B1C pilot code with the secondary pilot code
void beidou_b1cp_code_gen_complex_secondary(own::span<std::complex<float>> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp_secondary(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
       }
}


/*
* Generates a float version of the B1C pilot primary code
*/
void beidou_b1cp_code_gen_float_secondary(own::span<float> _dest, uint32_t _prn)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp_secondary(own::span<int32_t>(_code, _code_length), _prn);
       }

   for (uint32_t ii = 0; ii < _code_length; ii++)
       {
           _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
       }
}


/*
*  Generates complex BEIDOU B1C pilot code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
*/
void beidou_b1cp_code_gen_complex_sampled_secondary(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   uint32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
   int32_t _code[_code_length];

   if (_prn > 0 and _prn <= 63)
       {
           make_b1cp_secondary(own::span<int32_t>(_code, _code_length), _prn);
       }

   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;

   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length)));

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // code chip period in sec

   for (int32_t ii = 0; ii < _samples_per_code; ii++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C code values -------------------------
           _code_value_index = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1Cp code -----------------------
           if (ii == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_code_length - 1]);
               }
           else
               {
                   _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_code_value_index]);  //repeat the chip -> upsample
               }
       }
}


/*
*  Generates complex BEIDOU B1C data+pilot code for the desired SV ID and sampled to specific sampling frequency
*/
void beidou_b1c_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   uint32_t _code_length_data = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   uint32_t _code_length_pilot = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _code_pilot[_code_length_pilot];
   int32_t _code_data[_code_length_data];

   if (_prn > 0 and _prn < 63)
       {
           make_b1cp(own::span<int32_t>(_code_pilot, _code_length_pilot), _prn);
           make_b1cd(own::span<int32_t>(_code_data, _code_length_data), _prn);
       }

   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;
   const int32_t _code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;

   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length)));

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // C/A chip period in sec

   //float aux;
   for (int32_t ii = 0; ii < _samples_per_code; ii++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C pilot code values -------------------------
           _code_value_index = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1C code -----------------------
           if (ii == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_code_length - 1], 1.0 - 2.0 * _code_pilot[_code_length - 1]);
               }
           else
               {
                   _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_code_value_index], 1.0 - 2.0 * _code_pilot[_code_value_index]);  //repeat the chip -> upsample
               }
       }
}


//================================================BDS_B1C_BOC_Generation===================================================


void beidou_b1c_data_sinboc_11_gen_int(own::span<int> _dest, own::span<const int> _prn)
{
   const uint32_t _length_in = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   auto _period = static_cast<uint32_t>(_dest.size() / _length_in);

   for (uint32_t i = 0; i < _length_in; i++)
       {
           for (uint32_t j = 0; j < (_period / 2); j++)
               {
                   _dest[i * _period + j] = _prn[i];
               }
           for (uint32_t j = (_period / 2); j < _period; j++)
               {
                   _dest[i * _period + j] = -_prn[i];
               }
       }
}


void beidou_b1c_pilot_sinboc_11_gen_int(own::span<int> _dest, own::span<const int> _prn)
{
   const uint32_t _length_in = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   auto _period = static_cast<uint32_t>(_dest.size() / _length_in);

   for (uint32_t i = 0; i < _length_in; i++)
       {
           for (uint32_t j = 0; j < (_period / 2); j++)
               {
                   _dest[i * _period + j] = _prn[i];
               }
           for (uint32_t j = (_period / 2); j < _period; j++)
               {
                   _dest[i * _period + j] = -_prn[i];
               }
       }
}


void beidou_b1c_pilot_sinboc_61_gen_int(own::span<int> _dest, own::span<const int> _prn)
{
   const uint32_t _length_in = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   auto _period = static_cast<uint32_t>(_dest.size() / _length_in);

   for (uint32_t i = 0; i < _length_in; i++)
       {
           for (uint32_t j = 0; j < _period; j += 2)
               {
                   _dest[i * _period + j] = _prn[i];
               }
           for (uint32_t j = 1; j < _period; j += 2)
               {
                   _dest[i * _period + j] = -_prn[i];
               }
       }
}


//! Generates float version of sine BOC(1,1) modulated Data Code
void beidou_b1cd_gen_float_11(own::span<float> _dest, uint32_t _prn)
{
   /*
           *  PROCEDURE:
           *  1. Generate Beidou B1C Data Code
           *  2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code
           *  3. Multiply the output of Sine BOC(1,1) with the Power Ratio
    */
   uint32_t _primary_code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t b1c_data_primary_code_chips[_primary_code_length];

   // 1. Generate Beidou B1C Data Code
   make_b1cd(own::span<int32_t>(b1c_data_primary_code_chips, _primary_code_length), _prn);

   //In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
   //M = number of subchips per chip is calculated as, M=2*n/m
   //Here, 12 is the result of the sub chips after the BOC is applied
   const uint32_t _code_length = 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;

   //Value of alpha is according to equation 4-11 in ICD which is data component and in Real part of the equation,
   //SB1C(t)=(1/2*DB1C_data(t))*(CB1C_data(t))*(sign(sin(2πfsc_B1C_at)))+(sqrt(1.0 / 11.0)*(CB1C_pilot(t))*(sign(sin(2πfsc_B1C_bt)))+j(sqrt(29.0 / 44.0)*(CB1C_pilot(t)*(sign(sin(2πfsc_B1C_t)))
   const float alpha = (1.0 / 2.0);

   int32_t sinboc_11[12 * BEIDOU_B1C_CODE_LENGTH_CHIPS] = {0};  //  _code_length not accepted by Clang
   own::span<int32_t> sinboc_11_(sinboc_11, _code_length);

   // 2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code
   beidou_b1c_data_sinboc_11_gen_int(sinboc_11_, own::span<int>(b1c_data_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)));  //generate sinboc(1,1) 12 samples per chip
    // 3. Multiply the output of Sine BOC(1,1) with the Power Ratio
    //TODO Known issue: _dest size does not match the range of the loop
   for (uint32_t i = 0; i < _code_length; i++)
       {
           _dest[i] = alpha * static_cast<float>(sinboc_11[i]);
       }
}


//! Generates complex version of sine BOC(1,1) modulated Data Code
void beidou_b1cd_code_gen_complex_sampled_boc_11(own::span<std::complex<float>> _dest,
   uint32_t _prn, int32_t _fs)
{
   int32_t _samples_per_code, _samples_per_chip, _code_value_index;
   int32_t _boc_code_length;
   float _ts;
   float _tc;

   const int32_t _code_freq_basis = BEIDOU_B1C_CODE_RATE_CPS;  // Hz

   uint32_t _ranging_code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t _b1c_data_ranging_code_chips[_ranging_code_length];
   own::span<int32_t> _b1c_data_ranging_code_span(_b1c_data_ranging_code_chips, _ranging_code_length);

   // 1. Generate Beidou B1C Data Code
   make_b1cd(_b1c_data_ranging_code_span, _prn);

   // Find number of samples per spreading code, samples per chip, and delay
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(_code_freq_basis) / static_cast<double>(_ranging_code_length)));
   _samples_per_chip = 2;

   // Modulated code with sin BOC values
   _boc_code_length = _samples_per_chip * _ranging_code_length;
   float _b1c_data_boc_code_chips[_boc_code_length];
   own::span<float> _b1c_data_boc_code_span(_b1c_data_boc_code_chips, _boc_code_length);

   // Generate code with BOC modulation
   beidou_b1cd_gen_float_11(_b1c_data_boc_code_span, _prn);

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);               // Sampling period in sec
   _tc = 1.0 / static_cast<float>(_code_freq_basis);  // code chip period in sec


   for (int32_t i = 0; i < _samples_per_code; i++)
       {
           // Make index array to read B1C code values
           _code_value_index = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

           // Make the digitized version of the B1Cd code
           if (i == _samples_per_code - 1)
               {
                   // Correct the last index (due to number rounding issues)
                   _dest[i] = std::complex<float>(1.0 - 2.0 * _b1c_data_boc_code_span[_boc_code_length - 1], 0.0);
               }
           else
               {
                   //repeat the chip -> upsample
                   _dest[i] = std::complex<float>(1.0 - 2.0 * _b1c_data_boc_code_span[_code_value_index], 0.0);
               }
       }
}


//--------------------------------------------------BOC_FOR_PILOT_COMPONENT----------------------------------------------------

//! Generate BOC for first Pilot component which is in Real part
void beidou_b1cp_gen_float_61(own::span<float> _dest, uint32_t _prn)
{
   /*
           *  PROCEDURE:
           *  1. Generate Beidou B1C Pilot Code
           *  2. Apply Sine BOC(6,1) on generated Beidou B1C Pilot Code
           *  3. Multiply the output of Sine BOC(6,1) with the Power Ratio
    */
   uint32_t _primary_code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t b1c_pilot_primary_code_chips[_primary_code_length];

   // 1. Generate Beidou B1C Pilot Code
   make_b1cp(own::span<int32_t>(b1c_pilot_primary_code_chips, _primary_code_length), _prn);

   //In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
   //M = number of subchips per chip is calculated as, M=2*n/m
   //Here, 12 is the result of the sub chips after the BOC is applied
   const uint32_t _code_length = 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;

   //Value of alpha is according to equation 4-11 in ICD which is pilot component and in Real part of the equation,
   //SB1C(t)=(1/2*DB1C_data(t))*(CB1C_data(t))*(sign(sin(2πfsc_B1C_at)))+(sqrt(1.0 / 11.0)*(CB1C_pilot(t))*(sign(sin(2πfsc_B1C_bt)))+j(sqrt(29.0 / 44.0)*(CB1C_pilot(t)*(sign(sin(2πfsc_B1C_t)))
   const float alpha = sqrt(1.0 / 11.0);  // Power Ratio

   int32_t sinboc_61[12 * BEIDOU_B1C_CODE_LENGTH_CHIPS] = {0};  //  _code_length not accepted by Clang
   own::span<int32_t> sinboc_61_(sinboc_61, _code_length);

   // 2. Apply Sine BOC(6,1) on generated Beidou B1C Pilot Code
   beidou_b1c_pilot_sinboc_61_gen_int(sinboc_61_, own::span<int>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)));  //generate sinboc(6,1) 12 samples per chip

   // 3. Multiply the output of Sine BOC(6,1) with the Power Ratio
   for (uint32_t i = 0; i < _code_length; i++)
       {
           _dest[i] = alpha * static_cast<float>(sinboc_61[i]);
       }
}


//! Generate BOC for second Pilot component which is in Imaginary part
void beidou_b1cp_gen_float_11(own::span<float> _dest, uint32_t _prn)
{
   /*
           *  PROCEDURE:
           *  1. Generate Beidou B1C Pilot Code
           *  2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code(This Component is in Imaginary part of signal eqation)
           *  3. Multiply the output of Sine BOC(6,1) with the Power Ratio
    */
   uint32_t _primary_code_length = BEIDOU_B1C_CODE_LENGTH_CHIPS;
   int32_t b1c_pilot_primary_code_chips[_primary_code_length];

   // 1. Generate Beidou B1C Pilot Code
   make_b1cp(own::span<int32_t>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)), _prn);

   //In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
   //M = number of subchips per chip is calculated as, M=2*n/m
   //Here, 12 is the result of the sub chips after the BOC is applied
   const uint32_t _code_length = 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;

   //Value of beta is according to equation 4-11 in ICD
   const float beta = sqrt(29.0 / 44.0);

   int32_t sinboc_11[12 * BEIDOU_B1C_CODE_LENGTH_CHIPS] = {0};  //  _code_length not accepted by Clang
   own::span<int32_t> sinboc_11_(sinboc_11, _code_length);

   // 2. Apply Sine BOC(1,1) on generated Beidou B1C Pilot Code
   beidou_b1c_pilot_sinboc_11_gen_int(sinboc_11_, own::span<int>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)));  //generate sinboc(1,1) 12 samples per chip

   // 3. Multiply the output of Sine BOC(1,1) with the Power Ratio
   for (uint32_t i = 0; i < _code_length; i++)
       {
           _dest[i] = beta * static_cast<float>(sinboc_11[i]);
       }
}


//! Generates complex version of both pilot components having sine BOC(6,1) which is in Real part and sine BOC(1,1) which is in Imaginary part
void beidou_b1cp_code_gen_complex_sampled_boc_61_11(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;
   float aux;

   //TODO: fix this _code_length_pilot definition that doesn't match _dest size
   uint32_t _code_length_pilot = 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;
   float _code_pilot_real[_code_length_pilot];
   float _code_pilot_imag[_code_length_pilot];


   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length_pilot)));

   // Generate codes
   own::span<float> real_code_span(_code_pilot_real, _code_length_pilot);
   beidou_b1cp_gen_float_61(real_code_span, _prn);

   own::span<float> imaginary_code_span(_code_pilot_imag, _code_length_pilot);
   beidou_b1cp_gen_float_11(imaginary_code_span, _prn);

   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // code chip period in sec


   for (int32_t i = 0; i < _samples_per_code; i++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C code values -------------------------
           aux = (_ts * (static_cast<float>(i) + 1)) / _tc;
           _code_value_index = AUX_CEIL(aux) - 1;
           //_code_value_index = std::ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1Cd code -----------------------
           if (i == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_code_length_pilot - 1], 1.0 - 2.0 * imaginary_code_span[_code_length_pilot - 1]);
               }
           else
               {
                   //TODO: _dest is not defined to store the complete pilot code, but only a period of the data component
                   _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_code_value_index], 1.0 - 2.0 * imaginary_code_span[_code_value_index]);  //repeat the chip -> upsample
               }
       }
}


/*
* Generates complex version of data+pilot components as follows
* Data component(in ICD Equation 4-11) in Sine BOC(1,1) and is in Real part
* First Pilot component(in ICD Equation 4-11) in Sine BOC(6,1) and is in Real part
* Second Pilot component(in ICD Equation 4-11) in Sine BOC(1,1) and is in Imaginary part
*/
void beidou_b1c_code_gen_complex_sampled_boc(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
   //uint32_t _code_length =12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;
   //int32_t _code[_code_length];

   int32_t _samples_per_code, _code_value_index;
   float _ts;
   float _tc;
   const int32_t _code_length = 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS;

   //--- Find number of samples per spreading code ----------------------------
   _samples_per_code = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1C_CODE_RATE_CPS) / static_cast<double>(_code_length)));

   //Generating Data Component which is in Real Part
   auto* real_code_data = static_cast<float*>(volk_gnsssdr_malloc(_samples_per_code * sizeof(float), volk_gnsssdr_get_alignment()));
   own::span<float> real_code_span_data(real_code_data, _samples_per_code);
   beidou_b1cd_gen_float_11(real_code_span_data, _prn);


   //Generating Pilot Component which is in Real Part
   auto* real_code_pilot = static_cast<float*>(volk_gnsssdr_malloc(_samples_per_code * sizeof(float), volk_gnsssdr_get_alignment()));
   own::span<float> real_code_span_pilot(real_code_pilot, _samples_per_code);
   beidou_b1cp_gen_float_61(real_code_span_pilot, _prn);


   //Generating Pilot Component which is in Imaginary Part
   auto* imaginary_code_pilot = static_cast<float*>(volk_gnsssdr_malloc(_samples_per_code * sizeof(float), volk_gnsssdr_get_alignment()));
   own::span<float> imaginary_code_span_pilot(imaginary_code_pilot, _samples_per_code);
   beidou_b1cp_gen_float_11(imaginary_code_span_pilot, _prn);


   //XORing the output of two Real parts to put in one Real Part
   auto* real_code = static_cast<float*>(volk_gnsssdr_malloc(_samples_per_code * sizeof(float), volk_gnsssdr_get_alignment()));
   own::span<float> real_code_span(real_code, _samples_per_code);

   for (uint32_t m = 0; m < 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS; m++)
       {
           for (uint32_t n = 0; n < 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS; n++)
               {
                   real_code_span[n + m * 12 * BEIDOU_B1C_CODE_LENGTH_CHIPS] = real_code_span_data[n] + real_code_span_pilot[m];
               }
       }


   //--- Find time constants --------------------------------------------------
   _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
   _tc = 1.0 / static_cast<float>(BEIDOU_B1C_CODE_RATE_CPS);  // code chip period in sec


   for (int32_t i = 0; i < _samples_per_code; i++)
       {
           //=== Digitizing =======================================================

           //--- Make index array to read B1C code values -------------------------
           _code_value_index = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

           //--- Make the digitized version of the B1Cd code -----------------------
           if (i == _samples_per_code - 1)
               {
                   //--- Correct the last index (due to number rounding issues) -----------
                   _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_code_length - 1], 1.0 - 2.0 * imaginary_code_span_pilot[_code_length - 1]);
               }
           else
               {
                   _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_code_value_index], 1.0 - 2.0 * imaginary_code_span_pilot[_code_value_index - 1]);  //repeat the chip -> upsample
               }
       }
   volk_gnsssdr_free(real_code_data);
   volk_gnsssdr_free(real_code_pilot);
   volk_gnsssdr_free(imaginary_code_pilot);
   volk_gnsssdr_free(real_code);
}


//! Generates Data code required at the time of tracking(followed approach like Galileo E1)
void beidou_b1cd_code_gen_sinboc11_float(own::span<float> _dest, uint32_t _prn)
{
   const auto _code_length = static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
   std::array<int32_t, BEIDOU_B1C_CODE_LENGTH_CHIPS> primary_code_b1c_chips{};                        // _code_length not accepted by Clang
   make_b1cd(own::span<int32_t>(primary_code_b1c_chips.data(), BEIDOU_B1C_CODE_LENGTH_CHIPS), _prn);  //generate beidou B1C code, 1 sample per chip
   for (uint32_t i = 0; i < _code_length; i++)
       {
           _dest[2 * i] = static_cast<float>(primary_code_b1c_chips[i]);
           _dest[2 * i + 1] = -_dest[2 * i];
       }
}


//! Generates pilot code required at the time of tracking(followed approach like Galileo E1)
void beidou_b1cp_code_gen_sinboc11_float(own::span<float> _dest, uint32_t _prn)
{
   const auto _code_length = static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
   std::array<int32_t, BEIDOU_B1C_CODE_LENGTH_CHIPS> primary_code_b1c_chips{};                        // _code_length not accepted by Clang
   make_b1cp(own::span<int32_t>(primary_code_b1c_chips.data(), BEIDOU_B1C_CODE_LENGTH_CHIPS), _prn);  //generate beidou B1C code, 1 sample per chip
   for (uint32_t i = 0; i < _code_length; i++)
       {
           _dest[2 * i] = static_cast<float>(primary_code_b1c_chips[i]);
           _dest[2 * i + 1] = -_dest[2 * i];
       }
}
