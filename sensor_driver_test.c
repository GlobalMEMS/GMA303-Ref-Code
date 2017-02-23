/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Date : 2016/09/09
 *
 * Revision : 1.0.0
 *
 * Usage: GMA303 Sensor Driver Test
 *
 ****************************************************************************
 * @section License
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
 
/*! @file sensor_driver_test.c
 *  @brief  GMA303 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include <math.h>
#include "gma303.h"
#include "gSensor_autoNil.h"

#define DELAY_MS(ms)	//.....     /* Add your time delay function here */
#define RADIAN_TO_DEGREE            (180. / 3.14159265358979323846) //1 radian = 180/pi degree

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  u8 c;
  bus_support_t gma303_bus;
  raw_data_xyzt_t rawData;
  raw_data_xyzt_t offsetData;
  raw_data_xyzt_t calibData;
  float fTilt_degree;

  /* Add your HW initialization code here
    ...
    ...
    ...
    ...
  */
	
  /* GMA303 I2C bus setup */
  bus_init_I2C(&gma303_bus, GMA303_7BIT_I2C_ADDR);
  gma303_bus_init(&gma303_bus);

  /* GMA303 soft reset */
  gma303_soft_reset();

  /* GMA303 initialization */
  gma303_initialization();
	
  /* GMA303 Offset AutoNil */
  printf("Place and hold g-sensor in level for offset AutoNil.\n"); 
  printf("Press y when ready.\n");
  do{
    c = getchar();
  }while(c != 'y' && c != 'Y');

  //Conduct g-sensor AutoNil, g is along the Z-axis
  gSensorAutoNil(gma303_read_data_xyz, AUTONIL_AUTO + AUTONIL_Z, GMA303_RAW_DATA_SENSITIVITY, &offsetData);

  printf("Offset_XYZ=%d,%d%d\n", offsetData.u.x, offsetData.u.y, offsetData.u.z);
	
  for(;;)
    {

      /* Read XYZT data */
      gma303_read_data_xyzt(&rawData);

      //Offset compensation
      calibData.u.x = rawData.u.x - offsetData.u.x;
      calibData.u.y = rawData.u.y - offsetData.u.y;
      calibData.u.z = rawData.u.z - offsetData.u.z;

      //Tilt angle
      fTilt_degree = acos(calibData.u.z
			  / sqrt(calibData.u.x*calibData.u.x + calibData.u.y*calibData.u.y + calibData.u.z*calibData.u.z)
			  ) * RADIAN_TO_DEGREE;
 
      printf("Raw_XYZT=%d,%d,%d,%d\n", rawData.u.x, rawData.u.y, rawData.u.z, rawData.u.t);			
      printf("Calib_XYZ=%d,%d,%d\n", calibData.u.x, calibData.u.y, calibData.u.z);
      printf("Tilt=%d.%dDeg\n",
	     (s32)fTilt_degree,
	     abs((s32)((fTilt_degree - (s32)fTilt_degree)*100)));

      /* Delay 1 sec */
      DELAY_MS(1000);
		
    }
}
