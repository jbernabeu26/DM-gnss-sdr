/*!
 * \file beidou_cnav2_navigation_message_test.cc
 * \brief  This file implements tests for the decoding of the BEIDOU CNAV2 navigation message
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
 *
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

#include "gnss_signal_processing.h"
#include "beidou_cnav2_navigation_message.h"

/*!
 * \brief Testing CRC computation for BEIDOU CNAV2 data bits of a string
 * \test The provided string was generated with a version of MATLAB GNSS-SDR that
 * the author coded to perform proper decoding of BEIDOU CNAV2 signals.
 */
TEST(BeidouCnav2NavigationMessageTest, CRCTestSuccess)
{
    // Variables declarations in code
    bool test_result;
    std::bitset<BEIDOU_CNAV2_STRING_BITS> string_bits(std::string("011110001010100001001011110001000101000010111111111110001000110101001100110000000000011110010110001011111111111000001000011100010100000110001100000000000111001101111111111000001101101111000100111101000000000011111010100101100110011111110011010110011100101111001111011001001001100111111010101000000101001110101000110101000000101011001010111101001110001000100001101100001100000101010110010100101000101100110011010100001001011010000010110010110010001001100110101111011100001110010101010001110010111111101110000110110111101110101100010110001101111010010110001110110110110000011001"));
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    cnav2_nav_message.reset();

    // Call function to test
    test_result = cnav2_nav_message.CRC_test(string_bits);

    // Check results in unit test assetions
    ASSERT_TRUE(test_result);
}


/*!
 * \brief Testing CRC computation for BEIDOU CNAV2 data bits of a string
 * \test The provided string was generated with a version of MATLAB GNSS-SDR that
 * the author coded to perform proper decoding of BEIDOU CNAV2 signals.
 */
TEST(BeidouCnav2NavigationMessageTest, CRCTestFailure)
{
    // Variables declarations in code
    bool test_result;
    // Constructor of string to bitset will flip the order of the bits. Needed for CRC computation
    std::bitset<BEIDOU_CNAV2_STRING_BITS> string_bits(std::string("011110001010100001001011110001000101000010111111111110001000110101001100110000000000011110010110001011111111111000001000011100010100000110001100000000000111001101111111111000001101101111000100111101000000000011111010100101100110011111110011010110011100101111001111011001001001100111111010101000000101001110101000110101000000101011001010111101001110001000100001101100001100000101010110010100101000101100110011010100001001011010000010110010110010001001100110101111011100001110010101010001110010111111101110000110110111101110101100010110001101111010010110001110110110110000011000"));
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    cnav2_nav_message.reset();

    // Call function to test
    test_result = cnav2_nav_message.CRC_test(string_bits);

    // Check results in unit test assetions
    ASSERT_FALSE(test_result);
}


/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String1Decoder)
{
	// Message Type 10
    // Variable declarations
    std::string str1("011110001010100001001011110001000101000010111111111110001000110101001100110000000000011110010110001011111111111000001000011100010100000110001100000000000111001101111111111000001101101111000100111101000000000011111010100101100110011111110011010110011100101111001111011001001001100111111010101000000101001110101000110101000000101011001010111101001110001000100001101100001100000101010110010100101000101100110011010100001001011010000010110010110010001001100110101111011100001110010101010001110010111111101110000110110111101110101100010110001101111010010110001110110110110000011001");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407763;
    cnav2_ephemeris.WN = 645;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;
    cnav2_ephemeris.IODE = 17;
    cnav2_ephemeris.t_oe = 406800;
    cnav2_ephemeris.SatType = 3;
    cnav2_ephemeris.dA = 60.691406250000000;
    cnav2_ephemeris.A_dot = -0.007683753967285;
    cnav2_ephemeris.dn_0 = 0.000000001169780716736568;
    cnav2_ephemeris.dn_0_dot = 0.00000000000002563921297493721;
    cnav2_ephemeris.M_0 = -0.030411646468565;
    cnav2_ephemeris.e = 0.0004.779577138833702;
    cnav2_ephemeris.omega = -0.049411069834605;

    // Call target test method
    cnav2_nav_message.string_decoder(str1);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.WN - cnav2_nav_message.cnav2_ephemeris.WN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.IODE - cnav2_nav_message.cnav2_ephemeris.IODE < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.t_oe - cnav2_nav_message.cnav2_ephemeris.t_oe < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SatType - cnav2_nav_message.cnav2_ephemeris.SatType < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.dA - cnav2_nav_message.cnav2_ephemeris.dA < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.A_dot - cnav2_nav_message.cnav2_ephemeris.A_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.dn_0 - cnav2_nav_message.cnav2_ephemeris.dn_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.dn_0_dot - cnav2_nav_message.cnav2_ephemeris.dn_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.M_0 - cnav2_nav_message.cnav2_ephemeris.M_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.e - cnav2_nav_message.cnav2_ephemeris.e < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.omega - cnav2_nav_message.cnav2_ephemeris.omega < FLT_EPSILON);
}


/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String2Decoder)
{
	// Message Type 11
    // Variable declarations
    std::string str2("011110001011100001001011110000101111111111011110011000110100001010101111110001001110010010111111111000000000111011001111000101011101101001101100000000001010000000000000010011111111110011101010011001000000001111001100000011000000001100010011010111111101011110101111000010101000010110011001101110111101100101111100110110001011001000111011001111111111110000111101101010010100110000001011000100001000111111001110000110001001010100000001010101010101110111110001010101100011011101011110011100010100001000101001010001110101010000010100110111001110100010111111100010101110100101111000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407760;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.Omega_0 = 0.949616759549826;
    cnav2_ephemeris.i_0 = 0.30584704875946;
    cnav2_ephemeris.Omega_dot = -0.00000000221859863813734;
    cnav2_ephemeris.i_0_dot = -0.000000000136708422360243;
    cnav2_ephemeris.C_IS = 0.0000000372529029846191;
    cnav2_ephemeris.C_IC = 0.0000000176951289176941;
    cnav2_ephemeris.C_RS = -197.40234375;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00000961218029260635e;

    // Call target test method
    cnav2_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.Omega_0 - cnav2_nav_message.cnav2_ephemeris.Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0 - cnav2_nav_message.cnav2_ephemeris.i_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.Omega_dot - cnav2_nav_message.cnav2_ephemeris.Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0_dot - cnav2_nav_message.cnav2_ephemeris.i_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IS - cnav2_nav_message.cnav2_ephemeris.C_IS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IC - cnav2_nav_message.cnav2_ephemeris.C_IC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RS - cnav2_nav_message.cnav2_ephemeris.C_RS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RC - cnav2_nav_message.cnav2_ephemeris.C_RC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_US - cnav2_nav_message.cnav2_ephemeris.C_US < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_UC - cnav2_nav_message.cnav2_ephemeris.C_UC < FLT_EPSILON);
}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String3Decoder)
{
	// Message Type 30
    // Variable declarations
    std::string str3("011110011110100001001011110011101111111111101010011000111001010101111100110001111111010010101100011000000000000010100110111110101110011111101011100010101010001001100100101000101000001100000000000000001000000010100000100000000011000000000000000000000000000000000000111001000111110001000010101000000111101100011000101110101111000001001111001110100101101100110000101100001111110100100010101010101010011101001110111010001111101000111000111110010011111011110101010111100010101001110101010000011110100010110101011110101110101010101000011011000101110011101101111011010110011000100110");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407769;

    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_utc_model.t_oc = 406800;
    cnav2_utc_model.a_0 = 0.000874984136316925;
    cnav2_utc_model.a_1 = -0.0000000000412061496035676;
    cnav2_utc_model.a_2 = 0;

    cnav2_ephemeris.IODC = 333;
    cnav2_ephemeris.T_GDB2ap = -0.00000000954605638980865;
    cnav2_ephemeris.ISC_B2ad = 0.00000023603206500411;

    cnav2_ephemeris.alpha_1 = 10.625;
    cnav2_ephemeris.alpha_2 = 2.375;
    cnav2_ephemeris.alpha_3 = 4.625;
    cnav2_ephemeris.alpha_4 = 2.5;
    cnav2_ephemeris.alpha_5 = -3;
    cnav2_ephemeris.alpha_6 = 0;
    cnav2_ephemeris.alpha_7 = 0.5;
    cnav2_ephemeris.alpha_8 = 0.625;
    cnav2_ephemeris.alpha_9 = 0.5;
    cnav2_ephemeris.T_GDB1Cp = 0.00000000139698386192322;

    // Call target test method
    cnav2_nav_message.string_decoder(str3);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_utc_model.t_oc - cnav2_nav_message.cnav2_utc_model.t_oc < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_0 - cnav2_nav_message.cnav2_utc_model.a_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_1 - cnav2_nav_message.cnav2_utc_model.a_1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_2 - cnav2_nav_message.cnav2_utc_model.a_2 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.IODC - cnav2_nav_message.cnav2_ephemeris.IODC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.T_GDB2ap - cnav2_nav_message.cnav2_ephemeris.T_GDB2ap < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.ISC_B2ad - cnav2_nav_message.cnav2_ephemeris.ISC_B2ad < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_1 - cnav2_nav_message.cnav2_ephemeris.alpha_1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_2 - cnav2_nav_message.cnav2_ephemeris.alpha_2 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_3 - cnav2_nav_message.cnav2_ephemeris.alpha_3 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_4 - cnav2_nav_message.cnav2_ephemeris.alpha_4 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_5 - cnav2_nav_message.cnav2_ephemeris.alpha_5 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_6 - cnav2_nav_message.cnav2_ephemeris.alpha_6 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_7 - cnav2_nav_message.cnav2_ephemeris.alpha_7 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_8 - cnav2_nav_message.cnav2_ephemeris.alpha_8 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.alpha_9 - cnav2_nav_message.cnav2_ephemeris.alpha_9 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.T_GDB1Cp - cnav2_nav_message.cnav2_ephemeris.T_GDB1Cp < FLT_EPSILON);
}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String4Decoder)
{
	// Message Type 31
    // Variable declarations
    std::string str4("011110011111100001001011101111101111111111101010011000111001010101111100110001111111010010101100011000000000000010100110100010100001010011001100010101000111011000100000011100000000000111101111101100110111000101000000000010011011111010111001011001010000000000000000001010010100001111111111011110100011010000101011110110011001110100101100111100101110101001011011000010111001101110010111001111000001011010100110110001101000001100011010001111010111110010011000101011011001100000000110100001110110100101111001001111011111001110000111111110110000011001110101110100001111101011110011");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;
    Beidou_Cnav2_Almanac cnav2_almanac;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407757;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_utc_model.t_oc = 406800;
	cnav2_utc_model.a_0 = 0.000874984136316925;
	cnav2_utc_model.a_1 = -0.0000000000412061496035676;
	cnav2_utc_model.a_2 = 0;

	cnav2_ephemeris.IODC = 333;

	cnav2_almanac[5-1].WN_a = 645;
	cnav2_almanac[5-1].t_oa = 208896;
	cnav2_almanac[5-1].SatType = 1;
	cnav2_almanac[5-1].delta_A = 14848;
	cnav2_almanac[5-1].Omega_0 = -0.9375;
	cnav2_almanac[5-1].Phi_0 = 0.109375;
	cnav2_almanac[5-1].Health = 0;

	cnav2_almanac[7-1].WN_a = 645;
	cnav2_almanac[7-1].t_oa = 208896;
	cnav2_almanac[7-1].SatType = 2;
	cnav2_almanac[7-1].delta_A = -2560;
	cnav2_almanac[7-1].Omega_0 = 0.421875;
	cnav2_almanac[7-1].Phi_0 = -0.921875;
	cnav2_almanac[7-1].Health = 0;

	cnav2_almanac[9-1].WN_a = 645;
	cnav2_almanac[9-1].t_oa = 208896;
	cnav2_almanac[9-1].SatType = 2;
	cnav2_almanac[9-1].delta_A = -3072;
	cnav2_almanac[9-1].Omega_0 = -0.21875;
	cnav2_almanac[9-1].Phi_0 = -0.421875;
	cnav2_almanac[9-1].Health = 0;

    // Call target test method
    cnav2_nav_message.string_decoder(str4);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_utc_model.t_oc - cnav2_nav_message.cnav2_utc_model.t_oc < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_0 - cnav2_nav_message.cnav2_utc_model.a_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_1 - cnav2_nav_message.cnav2_utc_model.a_1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.a_2 - cnav2_nav_message.cnav2_utc_model.a_2 < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[5-1].WN_a - cnav2_nav_message.cnav2_almanac[5-1].WN_a < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].t_oa - cnav2_nav_message.cnav2_almanac[5-1].t_oa < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].SatType - cnav2_nav_message.cnav2_almanac[5-1].SatType < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].delta_A - cnav2_nav_message.cnav2_almanac[5-1].delta_A < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].Omega_0 - cnav2_nav_message.cnav2_almanac[5-1].Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].Phi_0 - cnav2_nav_message.cnav2_almanac[5-1].Phi_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[5-1].Health - cnav2_nav_message.cnav2_almanac[5-1].Health < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[7-1].WN_a - cnav2_nav_message.cnav2_almanac[7-1].WN_a < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].t_oa - cnav2_nav_message.cnav2_almanac[7-1].t_oa < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].SatType - cnav2_nav_message.cnav2_almanac[7-1].SatType < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].delta_A - cnav2_nav_message.cnav2_almanac[7-1].delta_A < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].Omega_0 - cnav2_nav_message.cnav2_almanac[7-1].Omega_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].Phi_0 - cnav2_nav_message.cnav2_almanac[7-1].Phi_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[7-1].Health - cnav2_nav_message.cnav2_almanac[7-1].Health < FLT_EPSILON);

	ASSERT_TRUE(cnav2_almanac[9-1].WN_a - cnav2_nav_message.cnav2_almanac[9-1].WN_a < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].t_oa - cnav2_nav_message.cnav2_almanac[9-1].t_oa < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].SatType - cnav2_nav_message.cnav2_almanac[9-1].SatType < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].delta_A - cnav2_nav_message.cnav2_almanac[9-1].delta_A < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].Omega_0 - cnav2_nav_message.cnav2_almanac[9-1].Omega_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].Phi_0 - cnav2_nav_message.cnav2_almanac[9-1].Phi_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[9-1].Health - cnav2_nav_message.cnav2_almanac[9-1].Health < FLT_EPSILON);
}


//TODO
//////
/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String2Decoder)
{
	// Message Type 11
    // Variable declarations
    std::string str2("011110001011100001001011110000101111111111011110011000110100001010101111110001001110010010111111111000000000111011001111000101011101101001101100000000001010000000000000010011111111110011101010011001000000001111001100000011000000001100010011010111111101011110101111000010101000010110011001101110111101100101111100110110001011001000111011001111111111110000111101101010010100110000001011000100001000111111001110000110001001010100000001010101010101110111110001010101100011011101011110011100010100001000101001010001110101010000010100110111001110100010111111100010101110100101111000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407760;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.Omega_0 = 0.949616759549826;
    cnav2_ephemeris.i_0 = 0.30584704875946;
    cnav2_ephemeris.Omega_dot = -0.00000000221859863813734;
    cnav2_ephemeris.i_0_dot = -0.000000000136708422360243;
    cnav2_ephemeris.C_IS = 0.0000000372529029846191;
    cnav2_ephemeris.C_IC = 0.0000000176951289176941;
    cnav2_ephemeris.C_RS = -197.40234375;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00000961218029260635e;

    // Call target test method
    cnav2_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.Omega_0 - cnav2_nav_message.cnav2_ephemeris.Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0 - cnav2_nav_message.cnav2_ephemeris.i_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.Omega_dot - cnav2_nav_message.cnav2_ephemeris.Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0_dot - cnav2_nav_message.cnav2_ephemeris.i_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IS - cnav2_nav_message.cnav2_ephemeris.C_IS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IC - cnav2_nav_message.cnav2_ephemeris.C_IC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RS - cnav2_nav_message.cnav2_ephemeris.C_RS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RC - cnav2_nav_message.cnav2_ephemeris.C_RC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_US - cnav2_nav_message.cnav2_ephemeris.C_US < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_UC - cnav2_nav_message.cnav2_ephemeris.C_UC < FLT_EPSILON);
}

//////
/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String2Decoder)
{
	// Message Type 11
    // Variable declarations
    std::string str2("011110001011100001001011110000101111111111011110011000110100001010101111110001001110010010111111111000000000111011001111000101011101101001101100000000001010000000000000010011111111110011101010011001000000001111001100000011000000001100010011010111111101011110101111000010101000010110011001101110111101100101111100110110001011001000111011001111111111110000111101101010010100110000001011000100001000111111001110000110001001010100000001010101010101110111110001010101100011011101011110011100010100001000101001010001110101010000010100110111001110100010111111100010101110100101111000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407760;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.Omega_0 = 0.949616759549826;
    cnav2_ephemeris.i_0 = 0.30584704875946;
    cnav2_ephemeris.Omega_dot = -0.00000000221859863813734;
    cnav2_ephemeris.i_0_dot = -0.000000000136708422360243;
    cnav2_ephemeris.C_IS = 0.0000000372529029846191;
    cnav2_ephemeris.C_IC = 0.0000000176951289176941;
    cnav2_ephemeris.C_RS = -197.40234375;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00000961218029260635e;

    // Call target test method
    cnav2_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.Omega_0 - cnav2_nav_message.cnav2_ephemeris.Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0 - cnav2_nav_message.cnav2_ephemeris.i_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.Omega_dot - cnav2_nav_message.cnav2_ephemeris.Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0_dot - cnav2_nav_message.cnav2_ephemeris.i_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IS - cnav2_nav_message.cnav2_ephemeris.C_IS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IC - cnav2_nav_message.cnav2_ephemeris.C_IC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RS - cnav2_nav_message.cnav2_ephemeris.C_RS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RC - cnav2_nav_message.cnav2_ephemeris.C_RC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_US - cnav2_nav_message.cnav2_ephemeris.C_US < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_UC - cnav2_nav_message.cnav2_ephemeris.C_UC < FLT_EPSILON);
}

//////
/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String2Decoder)
{
	// Message Type 11
    // Variable declarations
    std::string str2("011110001011100001001011110000101111111111011110011000110100001010101111110001001110010010111111111000000000111011001111000101011101101001101100000000001010000000000000010011111111110011101010011001000000001111001100000011000000001100010011010111111101011110101111000010101000010110011001101110111101100101111100110110001011001000111011001111111111110000111101101010010100110000001011000100001000111111001110000110001001010100000001010101010101110111110001010101100011011101011110011100010100001000101001010001110101010000010100110111001110100010111111100010101110100101111000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407760;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.Omega_0 = 0.949616759549826;
    cnav2_ephemeris.i_0 = 0.30584704875946;
    cnav2_ephemeris.Omega_dot = -0.00000000221859863813734;
    cnav2_ephemeris.i_0_dot = -0.000000000136708422360243;
    cnav2_ephemeris.C_IS = 0.0000000372529029846191;
    cnav2_ephemeris.C_IC = 0.0000000176951289176941;
    cnav2_ephemeris.C_RS = -197.40234375;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00000961218029260635e;

    // Call target test method
    cnav2_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.Omega_0 - cnav2_nav_message.cnav2_ephemeris.Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0 - cnav2_nav_message.cnav2_ephemeris.i_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.Omega_dot - cnav2_nav_message.cnav2_ephemeris.Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0_dot - cnav2_nav_message.cnav2_ephemeris.i_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IS - cnav2_nav_message.cnav2_ephemeris.C_IS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IC - cnav2_nav_message.cnav2_ephemeris.C_IC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RS - cnav2_nav_message.cnav2_ephemeris.C_RS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RC - cnav2_nav_message.cnav2_ephemeris.C_RC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_US - cnav2_nav_message.cnav2_ephemeris.C_US < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_UC - cnav2_nav_message.cnav2_ephemeris.C_UC < FLT_EPSILON);
}

//////
/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String2Decoder)
{
	// Message Type 11
    // Variable declarations
    std::string str2("011110001011100001001011110000101111111111011110011000110100001010101111110001001110010010111111111000000000111011001111000101011101101001101100000000001010000000000000010011111111110011101010011001000000001111001100000011000000001100010011010111111101011110101111000010101000010110011001101110111101100101111100110110001011001000111011001111111111110000111101101010010100110000001011000100001000111111001110000110001001010100000001010101010101110111110001010101100011011101011110011100010100001000101001010001110101010000010100110111001110100010111111100010101110100101111000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407760;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.Omega_0 = 0.949616759549826;
    cnav2_ephemeris.i_0 = 0.30584704875946;
    cnav2_ephemeris.Omega_dot = -0.00000000221859863813734;
    cnav2_ephemeris.i_0_dot = -0.000000000136708422360243;
    cnav2_ephemeris.C_IS = 0.0000000372529029846191;
    cnav2_ephemeris.C_IC = 0.0000000176951289176941;
    cnav2_ephemeris.C_RS = -197.40234375;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00000961218029260635e;

    // Call target test method
    cnav2_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(cnav2_ephemeris.PRN - cnav2_nav_message.cnav2_ephemeris.PRN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SOW - cnav2_nav_message.cnav2_ephemeris.SOW < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.HS - cnav2_nav_message.cnav2_ephemeris.HS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF - cnav2_nav_message.cnav2_ephemeris.DIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF - cnav2_nav_message.cnav2_ephemeris.SIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF - cnav2_nav_message.cnav2_ephemeris.AIF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISMAI - cnav2_nav_message.cnav2_ephemeris.SISMAI < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.DIF_B1C - cnav2_nav_message.cnav2_ephemeris.DIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SIF_B1C - cnav2_nav_message.cnav2_ephemeris.SIF_B1C < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.AIF_B1C - cnav2_nav_message.cnav2_ephemeris.AIF_B1C < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.Omega_0 - cnav2_nav_message.cnav2_ephemeris.Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0 - cnav2_nav_message.cnav2_ephemeris.i_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.Omega_dot - cnav2_nav_message.cnav2_ephemeris.Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.i_0_dot - cnav2_nav_message.cnav2_ephemeris.i_0_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IS - cnav2_nav_message.cnav2_ephemeris.C_IS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_IC - cnav2_nav_message.cnav2_ephemeris.C_IC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RS - cnav2_nav_message.cnav2_ephemeris.C_RS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_RC - cnav2_nav_message.cnav2_ephemeris.C_RC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_US - cnav2_nav_message.cnav2_ephemeris.C_US < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.C_UC - cnav2_nav_message.cnav2_ephemeris.C_UC < FLT_EPSILON);
}
