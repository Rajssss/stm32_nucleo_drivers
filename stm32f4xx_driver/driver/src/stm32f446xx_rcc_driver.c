
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
 * stm32f446xx.h
 *
 *  Created on: June 22, 2020
 *      Author: Rajssss@GitHub.com
 *
 *      Description: Reset and Clock Control Peripheral Access Layer Header for STM32F446xx.
 *
 *      This file contains:
 *          - Generic Function Definitions
 */


//includes
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx.h"


//AHB Prescaler Values
static const uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};

//APB1 Prescaler Values
static const uint8_t APB1_Prescaler[4] = {2, 4, 8, 16};




/***********************************************************************************
 * 					Peripheral_Clock-1 (APB1) Frequency Handler
 *
 * @fn: 		- 	RCC_GetPeriCLK1_Value
 *
 * @brief		-	This function returns the Peripheral clock frequency in Hz.
 *
 * @param[1]	-	None
 *
 * @return		-	uint32_t
 *
 * @Note		-
 *
 */
uint32_t RCC_GetPeriCLK1_Value(void)
{
	uint32_t SYSCLK, AHBP, APB1P;

	//Check for Clock source
	if(((RCC->CFGR >> 2) & 0x3) == 0x0)				//((RCC->CFGR >> 2) & 0x3) = Clock source
	{
		SYSCLK = HSI_CLK_FREQ;
	}
	else if(((RCC->CFGR >> 2) & 0x3) == 0x1)
	{
		SYSCLK = HSE_CLK_FREQ;
	}
	else if(((RCC->CFGR >> 2) & 0x3) == 0x2)
	{
		//TODO
	}
	else
	{
		//TODO
	}


	//Check AHB Prescalar Value
	if(((RCC->CFGR >> 4) & 0xF) < 8)				//((RCC->CFGR >> 4) & 0xF) = AHB Prescaler value
	{
		AHBP = 1;
	}
	else
	{
		AHBP = AHB_Prescaler[((RCC->CFGR >> 4) & 0xF) - 8];
	}


	//Check APB Low Speed (APB1 Bus) Prescaler Value
	if((((RCC->CFGR) >> 10) & 0x7) < 4)				//(((RCC->CFGR) >> 10) & 0x7) = APB Low Speed (APB1 Bus) Prescaler value
	{
		APB1P = 1;
	}
	else
	{
		APB1P = APB1_Prescaler[(((RCC->CFGR) >> 10) & 0x7) - 4];
	}


	return ((SYSCLK / AHBP) / APB1P);				//Peripheral Clock = (System_Clock / AHB Prescaler) / APB Low Speed Prescaler

}
