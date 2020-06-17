
/***********************************************************************************
 * This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ***********************************************************************************/

/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 16-June-2020
 *      Author: Rajssss@GitHub.com
 *      Description: I2C Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Data structures and the address mapping for all I2C Peripherals
 *           - I2C configuration registers declarations and bits definition
 *           - Macros to access various I2C configuration Registers
 *           - Macros to various I2C configuration Registers Mode and options
 *           - Function prototypes to perform various operation on a I2C Bus
 *
 */


#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include  "stm32f446xx.h"


/*****************************************************************************************
 * I2Cx Peripheral Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint32_t 		I2C_SCLSpeed;
	uint8_t 		I2C_DeviceAddress;
	uint8_t 		I2C_ACKControl;
	uint16_t		I2C_FMDutyCycle;

}I2Cx_Config_ty;



/*****************************************************************************************
 * I2Cx Handler structure.
 *****************************************************************************************/
typedef struct
{
	I2Cx_RegDef_ty *pI2Cx;
	I2Cx_Config_ty I2Cx_Config;

};

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
