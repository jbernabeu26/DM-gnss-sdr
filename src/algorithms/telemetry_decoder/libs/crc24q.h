/*!
 * \file crc24q.h
 * \brief  Implementation of the CRC-240 cyclic redundancy checksum
 * \author
 *
 * -------------------------------------------------------------------------
 *
 * This is an implementation of the CRC-24Q cyclic redundancy checksum
 * used by Qualcomm, RTCM104V3, and PGP 6.5.1. According to the RTCM104V3
 * standard, it uses the error polynomial
 *
 *    x^24+ x^23+ x^18+ x^17+ x^14+ x^11+ x^10+ x^7+ x^6+ x^5+ x^4+ x^3+ x+1
 *
 * This corresponds to a mask of 0x1864CFB.  For a primer on CRC theory,
 * including detailed discussion of how and why the error polynomial is
 * expressed by this mask, see <http://www.ross.net/crc/>.
 *
 * 1) It detects all single bit errors per 24-bit code word.
 * 2) It detects all double bit error combinations in a code word.
 * 3) It detects any odd number of errors.
 * 4) It detects any burst error for which the length of the burst is less than
 *    or equal to 24 bits.
 * 5) It detects most large error bursts with length greater than 24 bits;
 *    the odds of a false positive are at most 2^-23.
 *
 * This hash should not be considered cryptographically secure, but it
 * is extremely good at detecting noise errors.
 *
 * Note that this version has a seed of 0 wired in.  The RTCM104V3 standard
 * requires this.
 *
 * This file is Copyright (c) 2008,2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 *
 */
#ifndef _CRC24Q_H_
#define _CRC24Q_H_


extern void crc24q_sign(unsigned char *data, int len);

extern bool crc24q_check(unsigned char *data, int len);

extern unsigned crc24q_hash(unsigned char *data, int len);
#endif /* _CRC24Q_H_ */
