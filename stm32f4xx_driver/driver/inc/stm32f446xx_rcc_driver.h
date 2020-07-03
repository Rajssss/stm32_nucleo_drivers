
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
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: 02-Jul-2020
 *      Author: Rajssss@GitHub.com
 *
 *      Description: Reset and Clock Control Peripheral Access Layer Header for STM32F446xx.
 *
 *      This file contains:
 *      			- Reset and Clock Control Handlers
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"
//This returns the APB1 clock value
uint32_t RCC_GetPeriCLK1_Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPeriCLK2_Value(void);


#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
