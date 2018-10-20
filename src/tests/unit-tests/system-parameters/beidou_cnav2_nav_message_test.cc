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
 * This still has some bugs
 */
/*
TEST(BeidouCnav2NavigationMessageTest, CRCTestSuccess)
{
    // Variables declarations in code
    bool test_result;

    std::bitset<BEIDOU_CNAV2_STRING_BITS> string_bits(std::string("011110001010100001001011110001000101000010111111111110001000110101001100110000000000011110010110001011111111111000001000011100010100000110001100000000000111001101111111111000001101101111000100111101000000000011111010100101100110011111110011010110011100101111001111011001001001100111111010"));
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    cnav2_nav_message.reset();

    // Call function to test
    test_result = cnav2_nav_message.CRC_test(string_bits);

    // Check results in unit test assertions
    ASSERT_TRUE(test_result);

}
*/

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

    // Check results in unit test assertions
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
    cnav2_ephemeris.A_dot = -7.99232; //
    cnav2_ephemeris.dn_0 = 0.000000001169780716736568;
    cnav2_ephemeris.dn_0_dot = 0.00000000000002563921297493721;
    cnav2_ephemeris.M_0 = -0.9695883535; //
    cnav2_ephemeris.e = 0.0004779577138833702;
    cnav2_ephemeris.omega = -0.950589;

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
    cnav2_ephemeris.C_RS = -32570.59765625;
    cnav2_ephemeris.C_RC = 243.01171875;
    cnav2_ephemeris.C_US = 0.00000586546957492828;
    cnav2_ephemeris.C_UC = -0.00096695031970739365;

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
    Beidou_Cnav2_Almanac cnav2_almanac[3];

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

	cnav2_almanac[0].WN_a = 645;
	cnav2_almanac[0].t_oa = 208896;
	cnav2_almanac[0].SatType = 1;
	cnav2_almanac[0].delta_A = 14848;
	cnav2_almanac[0].Omega_0 = -0.9375;
	cnav2_almanac[0].Phi_0 = 0.109375;
	cnav2_almanac[0].Health = 0;

	cnav2_almanac[1].WN_a = 645;
	cnav2_almanac[1].t_oa = 208896;
	cnav2_almanac[1].SatType = 2;
	cnav2_almanac[1].delta_A = -62976;
	cnav2_almanac[1].Omega_0 = 0.421875;
	cnav2_almanac[1].Phi_0 = -0.921875;
	cnav2_almanac[1].Health = 0;

	cnav2_almanac[2].WN_a = 645;
	cnav2_almanac[2].t_oa = 208896;
	cnav2_almanac[2].SatType = 2;
	cnav2_almanac[2].delta_A = -62464;
	cnav2_almanac[2].Omega_0 = -0.78125;
	cnav2_almanac[2].Phi_0 = -0.578125;
	cnav2_almanac[2].Health = 0;

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

    ASSERT_TRUE(cnav2_ephemeris.IODC - cnav2_nav_message.cnav2_ephemeris.IODC < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[0].WN_a - cnav2_nav_message.cnav2_almanac[5-1].WN_a < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].t_oa - cnav2_nav_message.cnav2_almanac[5-1].t_oa < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].SatType - cnav2_nav_message.cnav2_almanac[5-1].SatType < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].delta_A - cnav2_nav_message.cnav2_almanac[5-1].delta_A < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Omega_0 - cnav2_nav_message.cnav2_almanac[5-1].Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Phi_0 - cnav2_nav_message.cnav2_almanac[5-1].Phi_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Health - cnav2_nav_message.cnav2_almanac[5-1].Health < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[1].WN_a - cnav2_nav_message.cnav2_almanac[7-1].WN_a < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].t_oa - cnav2_nav_message.cnav2_almanac[7-1].t_oa < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].SatType - cnav2_nav_message.cnav2_almanac[7-1].SatType < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].delta_A - cnav2_nav_message.cnav2_almanac[7-1].delta_A < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].Omega_0 - cnav2_nav_message.cnav2_almanac[7-1].Omega_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].Phi_0 - cnav2_nav_message.cnav2_almanac[7-1].Phi_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[1].Health - cnav2_nav_message.cnav2_almanac[7-1].Health < FLT_EPSILON);

	ASSERT_TRUE(cnav2_almanac[2].WN_a - cnav2_nav_message.cnav2_almanac[9-1].WN_a < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].t_oa - cnav2_nav_message.cnav2_almanac[9-1].t_oa < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].SatType - cnav2_nav_message.cnav2_almanac[9-1].SatType < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].delta_A - cnav2_nav_message.cnav2_almanac[9-1].delta_A < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].Omega_0 - cnav2_nav_message.cnav2_almanac[9-1].Omega_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].Phi_0 - cnav2_nav_message.cnav2_almanac[9-1].Phi_0 < FLT_EPSILON);
	ASSERT_TRUE(cnav2_almanac[2].Health - cnav2_nav_message.cnav2_almanac[9-1].Health < FLT_EPSILON);
}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String5Decoder)
{
	// Message Type 32
    // Variable declarations
    std::string str5("011110100000100001001011110110101111111111101010011000111001010101111100110001111111010010101100011000000000000010100110101011110111011000000000010010110101010000000000000000011001011111011110110000000000000000000000000111111111101001010011000000000000000000000000010011110111111001001100001010001101110010101001101101110000001101101001101000010010110100111100100001101110001010101111000011111100000001110101101110001111011010111100101001010001101011000011101001101101000100101001010101101111010100110100110110000000100110101001011010110010000010011010110111011011110000011100");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407778;
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

	cnav2_ephemeris.t_EOP = 388800;
	cnav2_ephemeris.PM_X = 0.00459766387939453;
	cnav2_ephemeris.PM_X_dot = 0;
	cnav2_ephemeris.PM_Y = 0.398310661315918;
	cnav2_ephemeris.PM_Y_dot = 0;
	cnav2_ephemeris.dUT1 = 0.124913394451141;
	cnav2_ephemeris.dUT1_dot = 0;

    // Call target test method
    cnav2_nav_message.string_decoder(str5);

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

    ASSERT_TRUE(cnav2_ephemeris.t_EOP - cnav2_nav_message.cnav2_ephemeris.t_EOP < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.PM_X - cnav2_nav_message.cnav2_ephemeris.PM_X < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.PM_X_dot - cnav2_nav_message.cnav2_ephemeris.PM_X_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.PM_Y - cnav2_nav_message.cnav2_ephemeris.PM_Y < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.PM_Y_dot - cnav2_nav_message.cnav2_ephemeris.PM_Y_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.dUT1 - cnav2_nav_message.cnav2_ephemeris.dUT1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.dUT1_dot - cnav2_nav_message.cnav2_ephemeris.dUT1_dot < FLT_EPSILON);

}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String6Decoder)
{
	// Message Type 33
    // Variable declarations
    std::string str6("011110100001100001001011111011101111111111101010011000111001010101111100110001111111010010101100011000000000000011101010011010100110101001101010011010100110101001101010011010100110010111111111111011110111100110000000001010011010001010000101010111010000000000000000110010110001111011000110010111101010110011000000111101101001010100101000110011101110000110001001011010101011101110001100000010110010010011101010000011111110100000101010010100001000111010001010001111010001010010101011001011011011010010100000101010001010110100110110110000001011011001011000100101000010010110100011");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;
    Beidou_Cnav2_Almanac cnav2_almanac[1];

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407793;
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

	cnav2_utc_model.GNSS_ID = 3;
	cnav2_utc_model.WN_0BGTO = 5429;
	cnav2_utc_model.t_0BGTO = 217936;
	cnav2_utc_model.A_0BGTO = 0.00000039642327465117;
	cnav2_utc_model.A_1BGTO = 0.000000000000755839835164807;
	cnav2_utc_model.A_2BGTO = 0.000000000000000000281214938488428;

	cnav2_almanac[0].SatType = 3;
	cnav2_almanac[0].delta_A = -65024;
	cnav2_almanac[0].Omega_0 = 0.953125;
	cnav2_almanac[0].Phi_0 = -0.796875;
	cnav2_almanac[0].Health = 0;
	cnav2_almanac[0].WN_a = 645;
	cnav2_almanac[0].t_oa = 380928;

	cnav2_ephemeris.IODC = 333;

    // Call target test method
    cnav2_nav_message.string_decoder(str6);

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

    ASSERT_TRUE(cnav2_utc_model.GNSS_ID - cnav2_nav_message.cnav2_utc_model.GNSS_ID < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.WN_0BGTO - cnav2_nav_message.cnav2_utc_model.WN_0BGTO < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.t_0BGTO - cnav2_nav_message.cnav2_utc_model.t_0BGTO < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.A_0BGTO - cnav2_nav_message.cnav2_utc_model.A_0BGTO < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.A_1BGTO - cnav2_nav_message.cnav2_utc_model.A_1BGTO < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.A_2BGTO - cnav2_nav_message.cnav2_utc_model.A_2BGTO < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.IODC - cnav2_nav_message.cnav2_ephemeris.IODC < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[0].SatType - cnav2_nav_message.cnav2_almanac[11-1].SatType < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].delta_A - cnav2_nav_message.cnav2_almanac[11-1].delta_A < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Omega_0 - cnav2_nav_message.cnav2_almanac[11-1].Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Phi_0 - cnav2_nav_message.cnav2_almanac[11-1].Phi_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Health - cnav2_nav_message.cnav2_almanac[11-1].Health < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].WN_a - cnav2_nav_message.cnav2_almanac[11-1].WN_a < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].t_oa - cnav2_nav_message.cnav2_almanac[11-1].t_oa < FLT_EPSILON);

}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String7Decoder)
{
	// Message Type 34
    // Variable declarations
    std::string str7("011110100010100001001011111100101111111111000000000000111100000010101001100011100101010111110011000111111101001010110001100000000000001010011010000011110101010000011011000000000000000010000110011110110110001010000010000100011110111000000100000000000000000000000000101011001101000110010100000101100000001010111101100010010101011101100011001101110001110101001001001001000001110110000000001001101000101000010100111111010100110011100001000111010111111010010100101001100001111000011101000000101110101100100111010100010100100110100111100100001001111110111101010001010001100100111110");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407796;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.t_op = 0;
	cnav2_ephemeris.SISAI_ocb = 15;
	cnav2_ephemeris.SISAI_oc1 = 0;
	cnav2_ephemeris.SISAI_oc2 = 0;

	cnav2_utc_model.t_oc = 406800;
	cnav2_utc_model.a_0 = 0.000874984136316925;
	cnav2_utc_model.a_1 = -0.0000000000412061496035676;
	cnav2_utc_model.a_2 = 0;

	cnav2_ephemeris.IODC = 333;

	cnav2_utc_model.A_0UTC = 0.0000000571017153561115;
	cnav2_utc_model.A_1UTC = 0.000000000000191846538655227;
	cnav2_utc_model.A_2UTC = 0;
	cnav2_utc_model.dt_LS = 4;
	cnav2_utc_model.t_ot = 212400;
	cnav2_utc_model.WN_ot = 642;
	cnav2_utc_model.WN_LSF = 573;
	cnav2_utc_model.DN = -2;
	cnav2_utc_model.dt_LSF = 4;

    // Call target test method
    cnav2_nav_message.string_decoder(str7);

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

    ASSERT_TRUE(cnav2_ephemeris.t_op - cnav2_nav_message.cnav2_ephemeris.t_op < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_ocb - cnav2_nav_message.cnav2_ephemeris.SISAI_ocb < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_oc1 - cnav2_nav_message.cnav2_ephemeris.SISAI_oc1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_oc2 - cnav2_nav_message.cnav2_ephemeris.SISAI_oc2 < FLT_EPSILON);

    ASSERT_TRUE(cnav2_utc_model.A_0UTC - cnav2_nav_message.cnav2_utc_model.A_0UTC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.A_1UTC - cnav2_nav_message.cnav2_utc_model.A_1UTC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.A_2UTC - cnav2_nav_message.cnav2_utc_model.A_2UTC < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.dt_LS - cnav2_nav_message.cnav2_utc_model.dt_LS < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.t_ot - cnav2_nav_message.cnav2_utc_model.t_ot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.WN_ot - cnav2_nav_message.cnav2_utc_model.WN_ot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.WN_LSF - cnav2_nav_message.cnav2_utc_model.WN_LSF < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.DN - cnav2_nav_message.cnav2_utc_model.DN < FLT_EPSILON);
    ASSERT_TRUE(cnav2_utc_model.dt_LSF - cnav2_nav_message.cnav2_utc_model.dt_LSF < FLT_EPSILON);
}

/*!
 * \brief Testing string decoding for BEIDOU CNAV2 messages
 * \test The provided string (str1.....str8) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of BEIDOU
 * CNAV2 signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(BeidouCnav2NavigationMessageTest, String8Decoder)
{
	// Message Type 40
    // Variable declarations
    std::string str8("011110101000100001001011101110101111111111011110000000000001111000000101001001101011011010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101011011010110110101101101000000000000000000000000000000000000000000110011100111111011010011010100000000101110000100001101110000111101101001010110010001101001111010110110100101000000000000010001111010011101111000010011101001000010110010101111110111100010010010011011010101000110101011011111010011011000000010111100100010001101011001110010001110100101010001101110111011001011000");
    Beidou_Cnav2_Navigation_Message cnav2_nav_message;
    Beidou_Cnav2_Ephemeris cnav2_ephemeris;
    Beidou_Cnav2_Utc_Model cnav2_utc_model;
    Beidou_Cnav2_Almanac cnav2_almanac[1];

    // Fill out ephemeris values for truth
    cnav2_ephemeris.PRN = 30;
    cnav2_ephemeris.SOW = 407754;
    cnav2_ephemeris.HS = 2;
    cnav2_ephemeris.DIF = 1;
    cnav2_ephemeris.SIF = 1;
    cnav2_ephemeris.AIF = 1;
    cnav2_ephemeris.SISMAI = 15;
    cnav2_ephemeris.DIF_B1C = 1;
    cnav2_ephemeris.SIF_B1C = 1;
    cnav2_ephemeris.AIF_B1C = 1;

    cnav2_ephemeris.SISAI_OE = 15;

    cnav2_ephemeris.t_op = 0;
    cnav2_ephemeris.SISAI_ocb = 15;
    cnav2_ephemeris.SISAI_oc1 = 0;
    cnav2_ephemeris.SISAI_oc2 = 0;

    cnav2_almanac[0].SatType = 0;
	cnav2_almanac[0].WN_a = 6874;
	cnav2_almanac[0].t_oa = 892928;
	cnav2_almanac[0].e = 0.026702880859375;
	cnav2_almanac[0].delta_i = -0.0423583984375;
	cnav2_almanac[0].sqrt_A = 5846.8125;
	cnav2_almanac[0].Omega_0 = 0.854888916015625;
	cnav2_almanac[0].Omega_dot = 0.000000101863406598568;
	cnav2_almanac[0].omega = 0.839202880859375;
	cnav2_almanac[0].M_0 = 0.839202880859375;
	cnav2_almanac[0].a_f0 = 0.000819206237792969;
	cnav2_almanac[0].a_f1 = 0.00000000265572452917695;
	cnav2_almanac[0].Health = 109;

    // Call target test method
    cnav2_nav_message.string_decoder(str8);

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

    ASSERT_TRUE(cnav2_ephemeris.SISAI_OE - cnav2_nav_message.cnav2_ephemeris.SISAI_OE < FLT_EPSILON);

    ASSERT_TRUE(cnav2_ephemeris.t_op - cnav2_nav_message.cnav2_ephemeris.t_op < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_ocb - cnav2_nav_message.cnav2_ephemeris.SISAI_ocb < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_oc1 - cnav2_nav_message.cnav2_ephemeris.SISAI_oc1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_ephemeris.SISAI_oc2 - cnav2_nav_message.cnav2_ephemeris.SISAI_oc2 < FLT_EPSILON);

    ASSERT_TRUE(cnav2_almanac[0].SatType - cnav2_nav_message.cnav2_almanac[41-1].SatType < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].WN_a - cnav2_nav_message.cnav2_almanac[41-1].WN_a < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].t_oa - cnav2_nav_message.cnav2_almanac[41-1].t_oa < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].e - cnav2_nav_message.cnav2_almanac[41-1].e < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].delta_i - cnav2_nav_message.cnav2_almanac[41-1].delta_i < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].sqrt_A - cnav2_nav_message.cnav2_almanac[41-1].sqrt_A < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Omega_0 - cnav2_nav_message.cnav2_almanac[41-1].Omega_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Omega_dot - cnav2_nav_message.cnav2_almanac[41-1].Omega_dot < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].omega - cnav2_nav_message.cnav2_almanac[41-1].omega < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].M_0 - cnav2_nav_message.cnav2_almanac[41-1].M_0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].a_f0 - cnav2_nav_message.cnav2_almanac[41-1].a_f0 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].a_f1 - cnav2_nav_message.cnav2_almanac[41-1].a_f1 < FLT_EPSILON);
    ASSERT_TRUE(cnav2_almanac[0].Health - cnav2_nav_message.cnav2_almanac[41-1].Health < FLT_EPSILON);

}
