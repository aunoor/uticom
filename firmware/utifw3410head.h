/* vi: ts=8 sw=8
 *
 * TI 3410 USB Serial Driver Firmware Header
 *
 * Copyright (C) 2005 Griffin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _TI_FW_3410_P_H_
#define _TI_FW_3410_P_H_

static unsigned char ti_fw_3410[] = {
/*  struct ImageHdr {
 *      WORD    wLength;
 *      BYTE    bCheckSum;
 * };
 */
0x0d, 0xce,	/* firmware image length excluding header, little endian */
0x00,
