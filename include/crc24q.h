/* Interface for CRC-24Q cyclic redundancy checksum code
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _CRC24Q_H_
#define _CRC24Q_H_

extern void crc24q_sign(unsigned char *data, int len);

extern bool crc24q_check(unsigned char *data, int len);

extern unsigned crc24q_hash(unsigned char *data, int len);
#endif  // _CRC24Q_H_

// vim: set expandtab shiftwidth=4
