
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
 * stm32f446xx_usart_driver.c
 *
 *  Created on: 21-Jul-2020
 *      Author: Rajssss@GitHub.com
 *      Description: USART/UART Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Function definitions to perform various operation on USART/UART Peripheral.
 *
 *
 */

//includes
#include "stm32f446xx_usart_driver.h"



/***********************************************************************************
 * 					 	USART/UART Clock Control Handler
 *
 * @fn: 		- 	USARTx_PeriClkControl
 *
 * @brief		-	This function Enables or Disables the peripheral clock of
 * 					the given USART/UART Peripheral
 *
 * @param[1]	-	Base Address of the USART/UART Peripheral
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void USARTx_PeriClkControl(USARTx_RegDef_ty *pUSARTx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}

}




/***********************************************************************************
 * 					 	USART/UART peripheral Control Handler
 *
 * @fn: 		- 	USARTx_PeripheralControl
 *
 * @brief		-	This function Enable or Disable the given USART/UART Peripheral
 *
 * @param[1]	-	Base Address of the USART/UART Peripheral
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void USARTx_PeripheralControl(USARTx_RegDef_ty *pUSARTx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}



