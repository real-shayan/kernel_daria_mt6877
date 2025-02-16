/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __IMX350GMS_EEPROM_H__
#define __IMX350GMS_EEPROM_H__

#include "kd_camera_typedef.h"

void imx350gms_read_SPC(BYTE * data);
void imx350gms_read_DCC(kal_uint16 addr, BYTE *data, kal_uint32 size);

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
			u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

extern int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
			     u8 *a_pRecvData,
			     u16 a_sizeRecvData, u16 i2cId, u16 timing);

extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);

#endif

