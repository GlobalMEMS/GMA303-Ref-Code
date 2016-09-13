/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : bus_support.h
 *
 * Date : 2016/05/20
 *
 * Revision : 1.0.0
 *
 * Usage: I2C/SPI bus support
 *
 ****************************************************************************
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/
 
/*! @file bus_support.h
 *  @brief  I2C/SPI bus read/write support file 
 *  @author Joseph FC Tseng
 */
 
#ifndef __BUS_SUPPORT_H__
#define __BUS_SUPPORT_H__
 
#include "type_support.h"

//I2C bus read write function definition
#define BUS_RD_FUNC_PTR s8(*bus_read)(u8, u8, u8*, u8)
#define BUS_WR_FUNC_PTR s8(*bus_write)(u8, u8, u8*, u8)
#define BUS_READ_FUNC(u8DevAddr, u8RegAddr, pu8RegData, u8Len) bus_read(u8DevAddr, u8RegAddr, pu8RegData, u8Len)
#define BUS_WRITE_FUNC(u8DevAddr, u8RegAddr, pu8RegData, u8Len) bus_write(u8DevAddr, u8RegAddr, pu8RegData, u8Len)

typedef struct {
		u8 u8DevAddr;
		BUS_WR_FUNC_PTR;
		BUS_RD_FUNC_PTR;
} bus_support_t;

 /*!
 * @brief Intialize to I2C bus. 
 * @brief Implementation dependent. Replace actual I2C function here.  
 *
 * @param pbus Pointer to the bus support type. Will be assigned the actual I2C R/W function and slave address
 * @param u8DevAddr 7-bit I2C slave address of the device
 *
 * @return Always return 0
 */
s8 bus_init_I2C(bus_support_t* pbus, u8 u8DevAddr);

 
 #endif //__BUS_SUPPORT_H__
 
