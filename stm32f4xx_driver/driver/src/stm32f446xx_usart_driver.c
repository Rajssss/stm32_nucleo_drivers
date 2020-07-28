
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





/***********************************************************************************
 * 					 		USART/UART Flag Status Handler
 *
 * @fn: 		- 	USARTx_GetFlagStatus
 *
 * @brief		-	Function which returns status of a flag of Status Register (SR).
 *
 * @param[1]	-	Base Address of the USART/UART Peripheral
 *
 * @param[2]	-	Flag value from @USART_FLAG_BITS
 *
 * @return		-	uint8_t
 *
 * @Note		-
 *
 */
uint8_t USARTx_GetFlagStatus(USARTx_RegDef_ty *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}





/***********************************************************************************
 * 					 		USART/UART Flag Status Handler
 *
 * @fn: 		- 	USARTx_ClearFlag
 *
 * @brief		-	Function to clear a specific flag of Status Register (SR).
 *
 * @param[1]	-	Base Address of the USART/UART Peripheral
 *
 * @param[2]	-	Flag value from @USART_FLAG_BITS
 *
 * @return		-	void
 *
 * @Note		-	Applicable only for RXNE,TC,LBD and CTS Flags.
 *
 */
void USARTx_ClearFlag(USARTx_RegDef_ty *pUSARTx, uint16_t FlagName)
{
	pUSARTx->SR &= ~FlagName;
}





/***********************************************************************************
 * 					 	  	 USART/UART Initialize Handler
 *
 * @fn: 		- 	USARTx_Init
 *
 * @brief		-	This function will configure the various Registers associated with that
 * 					USART/UART Peripheral with the values present in USARTx_Config_ty structure.
 *
 * @param[1]	-	Address of the USART/UART Handler Request.
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void USARTx_Init(USARTx_Handler_ty *pUSARTHandler)
{
	//Enable peripheral clock
	USARTx_PeriClkControl(pUSARTHandler->pUSARTx, ENABLE);

	//Configure USART/UART Rx/Tx as per USART_MODE
	if(pUSARTHandler->USARTx_Config.USART_Mode == USART_MODE_RX)
	{
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandler->USARTx_Config.USART_Mode == USART_MODE_TX)
	{
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandler->USARTx_Config.USART_Mode == USART_MODE_TXRX)
	{
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TE);
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_RE);
	}

	//Configure Word Length
	pUSARTHandler->pUSARTx->CR1 |= (pUSARTHandler->USARTx_Config.USART_WordLength << USART_CR1_M);

	//Configure Parity
	if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_EVEN)
	{
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
	}
	else if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_ODD)
	{
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_PS);
	}

	//Configure STOP Bits
	pUSARTHandler->pUSARTx->CR2 |= (pUSARTHandler->USARTx_Config.USART_StopBits_No << USART_CR2_STOP);

	//Configure HW Flow Control
	if(pUSARTHandler->USARTx_Config.USART_HWFlowControl == USART_HW_FLOW_CTS)
	{
		pUSARTHandler->pUSARTx->CR3 |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandler->USARTx_Config.USART_HWFlowControl == USART_HW_FLOW_RTS)
	{
		pUSARTHandler->pUSARTx->CR3 |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandler->USARTx_Config.USART_HWFlowControl == USART_HW_FLOW_CTS_RTS)
	{
		pUSARTHandler->pUSARTx->CR3 |= (1 << USART_CR3_CTSE);
		pUSARTHandler->pUSARTx->CR3 |= (1 << USART_CR3_RTSE);
	}

	//Configure Baudrate
	USARTx_BaudRate_Config(pUSARTHandler->pUSARTx, pUSARTHandler->USARTx_Config.USART_Baud);

}






/***********************************************************************************
 * 					 	USART/UART Baud Rate Config Handler
 *
 * @fn: 		- 	USARTx_BaudRate_Config
 *
 * @brief		-	This function Enables or Disables the peripheral clock of
 * 					the given USART/UART Peripheral
 *
 * @param[1]	-	Base Address of the USART/UART Peripheral
 *
 * @param[2]	-	Baud Rate.
 *
 * @return		-	void
 *
 * @Note		-	Applicable only in asynchronous mode.
 *
 */
void USARTx_BaudRate_Config(USARTx_RegDef_ty *pUSARTx, uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t USARTDIV_Mantissa, USARTDIV_Fraction, USARTDIV;

	//get PCLK value based on APB Bus Clock
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		PCLKx = RCC_GetPeriCLK2_Value();
	}
	else
	{
		PCLKx = RCC_GetPeriCLK1_Value();
	}

	//if OVER8=1
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//multiply by 100 to get rid of fraction part
		USARTDIV = (((float)PCLKx / (8 * BaudRate)) * 100);

		//Calculate Mantissa
		USARTDIV_Mantissa = USARTDIV/100;

		//Calculate Fraction
		USARTDIV_Fraction = (USARTDIV - (USARTDIV_Mantissa * 100));

		//Round Off the fraction
		//when OVER8=1, the DIV_Fraction3 bit is not considered and must be kept cleared.
		USARTDIV_Fraction = (((USARTDIV_Fraction * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		//multiply by 100 to get rid of fraction part
		USARTDIV = (((float)PCLKx / (16 * BaudRate)) * 100);

		//Calculate Mantissa
		USARTDIV_Mantissa = USARTDIV/100;

		//Calculate Fraction
		USARTDIV_Fraction = (USARTDIV - (USARTDIV_Mantissa * 100));

		//Round Off the fraction
		USARTDIV_Fraction = (((USARTDIV_Fraction * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	//Program Baudrate (BRR) Register
	pUSARTx->BRR |= ((USARTDIV_Mantissa << 0x4) + USARTDIV_Fraction);

}






/***********************************************************************************
 * 					 	USART/UART Tx Handler
 *
 * @fn: 		- 	USARTx_SendData
 *
 * @brief		-	This function handles the data transmission the given USART/UART
 * 					Peripheral.
 *
 * @param[1]	-	Base Address of the USART/UART Handler
 *
 * @param[2]	-	Transmission data Buffer
 *
 * @param[3]	-	Transmission data length
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void USARTx_SendData(USARTx_Handler_ty *pUSARTHandler, uint8_t *pTxBuffer, uint8_t length)
{
	//loop till length no of data are transferred
	for(uint32_t var=0; var < length; var++)
	{
		//wait till TXE is set
		while(! USARTx_GetFlagStatus(pUSARTHandler->pUSARTx, USART_FLAG_TXE));

		//check word length
		if(pUSARTHandler->USARTx_Config.USART_WordLength == USART_WORD_LEN_9BITS)
		{
			//extract 9-Bit and put it in DR
			pUSARTHandler->pUSARTx->DR = (*(uint16_t* )pTxBuffer & (uint16_t)0x01FF);

			//check whether parity is enable
			if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer += 2;
			}
			else
			{
				pTxBuffer++;
			}


		}
		else
		{
			//write 1-Byte to DR
			pUSARTHandler->pUSARTx->DR = *pTxBuffer & (uint8_t)0xFF;
			pTxBuffer++;
		}
	}

	//wait till TC is set in SR
	while(! USARTx_GetFlagStatus(pUSARTHandler->pUSARTx, USART_FLAG_TC));
}






/***********************************************************************************
 * 					 	USART/UART Rx Handler
 *
 * @fn: 		- 	USARTx_ReceiveData
 *
 * @brief		-	This function handles the data reception the given USART/UART
 * 					Peripheral.
 *
 * @param[1]	-	Base Address of the USART/UART Handler
 *
 * @param[2]	-	Receiving data Buffer
 *
 * @param[3]	-	Receiving  data length
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void USARTx_ReceiveData(USARTx_Handler_ty *pUSARTHandler, uint8_t *pRxBuffer, uint8_t length)
{
	//loop till length number of data is received
	for (uint32_t var = 0; var < length; ++var)
	{
		//wait till RXNE is set in SR
		while(! USARTx_GetFlagStatus(pUSARTHandler->pUSARTx, USART_FLAG_RXNE));

		//check word length
		if(pUSARTHandler->USARTx_Config.USART_WordLength == USART_WORD_LEN_9BITS)
		{
			//check parity is enable or not
			if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//Read 9-Bits from DR
				*((uint16_t*) pRxBuffer) = (pUSARTHandler->pUSARTx->DR & (uint16_t)0x1FF);
				pRxBuffer += 2;
			}
			else
			{
				//Read 8-Bits from DR
				*pRxBuffer = (pUSARTHandler->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}

		}
		else
		{
			//check parity is enable or not
			if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//Read 8-Bits from DR
				*pRxBuffer = (pUSARTHandler->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
			else
			{
				//Read 7-Bits from DR
				*pRxBuffer = (uint8_t)(pUSARTHandler->pUSARTx->DR & (uint8_t)0x7F);
				pRxBuffer++;
			}

		}

	}

}





