
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
 * 					 		USART/UART Flag Clear Handler
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

	//Enable USART Peripheral
	USARTx_PeripheralControl(pUSARTHandler->pUSARTx, ENABLE);

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





/***********************************************************************************
 * 					 		USART/UART Tx Interrupt Handler
 *
 * @fn: 		- 	USARTx_SendData_IT
 *
 * @brief		-	This function handles the interrupt based data transmission the given USART/UART
 * 					Peripheral.
 *
 * @param[1]	-	Base Address of the USART/UART Handler
 *
 * @param[2]	-	Transmission data Buffer
 *
 * @param[3]	-	Transmission data length
 *
 * @return		-	uint8_t
 *
 * @Note		-
 *
 */
uint8_t USARTx_SendData_IT(USARTx_Handler_ty *pUSARTHandler, uint8_t *pTxBuffer, uint8_t length)
{
	//check if busy in Tx
	if(pUSARTHandler->BusyTxState != USART_BUSY_TX)
	{
		pUSARTHandler->TxLen = length;
		pUSARTHandler->pTxBuffer = pTxBuffer;
		pUSARTHandler->BusyTxState = USART_BUSY_TX;

		//Enable Tx Interrupt
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable TC Interrupt
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

	}

	return pUSARTHandler->BusyTxState;

}







/***********************************************************************************
 * 					 		USART/UART Rx Interrupt Handler
 *
 * @fn: 		- 	USARTx_ReceiveData_IT
 *
 * @brief		-	This function handles the interrupt based data reception the given USART/UART
 * 					Peripheral.
 *
 * @param[1]	-	Base Address of the USART/UART Handler
 *
 * @param[2]	-	Receiving data Buffer
 *
 * @param[3]	-	Receiving  data length
 *
 * @return		-	uint8_t
 *
 * @Note		-
 *
 */
uint8_t USARTx_ReceiveData_IT(USARTx_Handler_ty *pUSARTHandler, uint8_t *pRxBuffer, uint8_t length)
{
	//check if Busy in Rx
	if(pUSARTHandler->BusyRxState != USART_BUSY_RX)
	{
		pUSARTHandler->RxLen = length;
		pUSARTHandler->pRxBuffer = pRxBuffer;
		pUSARTHandler->BusyRxState = USART_BUSY_RX;

		//Enable Rx Interrupt
		pUSARTHandler->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return pUSARTHandler->BusyRxState;
}







/***********************************************************************************
 * 					 		USART/UART Rx Interrupt Handler
 *
 * @fn: 		- 	USARTx_IRQPriorityConfig
 *
 * @brief		-	This function configures the interrupt priority of
 * 					given USART/UART Peripheral.
 *
 * @param[1]	-	IRQ number of the Interrupt
 *
 * @param[2]	-	Interrupt Priority
 *
 * @return		-	void
 *
 * @Note		-	1.In STM32F446RE, Number of priority bits implemented are 4 (MSB) out of 8 bits for each IRQ Number.
 * 					2.So we have to shift the desired priority vale left by 4 before finally writing it.
 *
 */
void USARTx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1.(IRQNumber / 4) will select the specific PR resister out of 60 PR registers then
	//3.(NVIC_PR_BASE_ADDR + (IRQNumber / 4) will finally jumps to the required base address of the PR register
	//4.(IRQNumber % 4) will select the specific bit section for a PR register.
	//5.((IRQNumber % 4) * 8) will select the 8 bit section range.
	//6.(8 - NVIC_NO_PR_BITS_IMPLEMENTED) will select the required addition shifts, as NVIC_NO_PR_BITS_IMPLEMENTED is vendor specific
	//7.(((IRQNumber % 4) * 8) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED)) will finally give the required shift amount.

	*(NVIC_PR_BASE_ADDR + (IRQNumber / 4)) |= (IRQPriority << (((IRQNumber % 4) * 8) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED)));
}






/***********************************************************************************
 * 					 	USART/UART Interrupt Config Handler
 *
 * @fn: 		- 	USARTx_IRQInterruptConfig
 *
 * @brief		-	This function configures the interrupt related configurations of
 * 					given USART/UART Peripheral.
 *
 * @param[1]	-	IRQ number of the Interrupt
 *
 * @param[2]	-	Control: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void USARTx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Control)
{
	if(Control == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
		else if(IRQNumber > 96 && IRQNumber < 128)
		{
			//ISER3
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}
		else if(IRQNumber > 128 && IRQNumber < 160)
		{
			//ISER4
			*NVIC_ISER4 |= (1 << (IRQNumber % 128));
		}
		else if(IRQNumber > 160 && IRQNumber < 192)
		{
			//ISER5
			*NVIC_ISER5 |= (1 << (IRQNumber % 160));
		}
		else if(IRQNumber > 192 && IRQNumber < 224)
		{
			//ISER6
			*NVIC_ISER6 |= (1 << (IRQNumber % 192));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ICER1
			*NVIC_ICER1 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//ICER2
			*NVIC_ICER2 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 96 && IRQNumber < 128)
		{
			//ICER3
			*NVIC_ICER3 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 128 && IRQNumber < 160)
		{
			//ICER4
			*NVIC_ICER4 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 160 && IRQNumber < 192)
		{
			//ICER5
			*NVIC_ICER5 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 192 && IRQNumber < 224)
		{
			//ICER6
			*NVIC_ICER6 |= (1 << IRQNumber);
		}
	}
}






/***********************************************************************************
 * 					 	USART/UART Interrupt Handler
 *
 * @fn: 		- 	USARTx_IRQHandling
 *
 * @brief		-	This function handles various USART/UART interrupts.
 *
 * @param[1]	-	Base Address of the USART/UART Handler
 *
 * @return		-	void
 *
 * @Note		-	TODO: Multibuffer mode (Noise Error, Parity Error, Break Flag, Framing Error)
 *
 */
void USARTx_IRQHandling(USARTx_Handler_ty *pUSARTHandler)
{

	/**** Interrupt due to TC flag ********************************/

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_TC)) && (pUSARTHandler->pUSARTx->CR1 & (1 << USART_CR1_TCIE)))
	{
		//TC is set so close Tx and repond Application via callback if Txlength is zero.
		if(pUSARTHandler->BusyTxState == USART_BUSY_TX)
		{
			//if TxLen is zero
			if(! pUSARTHandler->TxLen)
			{
				//clear TC Flag
				USARTx_ClearFlag(pUSARTHandler->pUSARTx, USART_FLAG_TC);

				//Disable TC Interrupt (Clear TCIE)
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Change state to Ready
				pUSARTHandler->BusyTxState = USART_READY;

				//Reset Buffer
				pUSARTHandler->pTxBuffer = NULL;

				//Reset TxLen
				pUSARTHandler->TxLen = RESET;

				//Notify the application
				USARTx_ApplicationEventCallback(pUSARTHandler, USART_EVENT_TX_CMPLT);

			}

		}

	}

	/**** Interrupt due to TXE flag *******************************/

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_TXE)) && (pUSARTHandler->pUSARTx->CR1 & (1 << USART_CR1_TXEIE)) )
	{
		if(pUSARTHandler->BusyTxState == USART_BUSY_TX)
		{
			//Start TX | keep sending till TxLen==0
			if(pUSARTHandler->TxLen > 0)
			{
				//check WORD length
				if(pUSARTHandler->USARTx_Config.USART_WordLength == USART_WORD_LEN_9BITS)
				{
					//load 9-Bits to DR
					pUSARTHandler->pUSARTx->DR = *((uint16_t *)pUSARTHandler->pTxBuffer) & ((uint16_t)0x01FF);

					//check PARITY configure
					if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						pUSARTHandler->pTxBuffer +=2;
						pUSARTHandler->TxLen -=2;
					}
					else
					{
						pUSARTHandler->pTxBuffer++;
						pUSARTHandler->TxLen--;
					}
				}
				else
				{
					pUSARTHandler->pUSARTx->DR = (*pUSARTHandler->pRxBuffer & (uint8_t)0xFF);
					pUSARTHandler->pTxBuffer++;
					pUSARTHandler->TxLen--;
				}
			}

			//if TxLen is zero, clear TXIE (Disable TXE Interrupt)
			if(pUSARTHandler->TxLen == 0)
			{
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/**** Interrupt due to RXNE flag ******************************/

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_RXNE)) && (pUSARTHandler->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE)))
	{
		if(pUSARTHandler->BusyRxState == USART_BUSY_RX)
		{

			if(pUSARTHandler->RxLen > 0)
			{
				if(pUSARTHandler->USARTx_Config.USART_WordLength == USART_WORD_LEN_9BITS)
				{
					//check PARITY config
					if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//Read 9-Bits from DR
						*((uint16_t *)pUSARTHandler->pRxBuffer) = (pUSARTHandler->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandler->pRxBuffer +=2;
						pUSARTHandler->RxLen -=2;
					}
					else
					{
						//Read 8-Bit from DR (1 bit is used for parity)
						*pUSARTHandler->pRxBuffer = (pUSARTHandler->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandler->pRxBuffer++;
						pUSARTHandler->RxLen--;
					}

				}
				else
				{
					//check PARITY config
					if(pUSARTHandler->USARTx_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//Read 8-Bits from DR
						*pUSARTHandler->pRxBuffer = (pUSARTHandler->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						//Read 7-Bit from DR (1 bit is used for parity)
						*pUSARTHandler->pRxBuffer = (pUSARTHandler->pUSARTx->DR & (uint8_t)0x7F);
					}

					pUSARTHandler->pRxBuffer++;
					pUSARTHandler->RxLen--;
				}

			}

			//If RxLen is zero, disable RXNE Interrupt and state to READY
			if(pUSARTHandler->RxLen == 0)
			{
				pUSARTHandler->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandler->BusyRxState = USART_READY;

				//Notify the Application
				USARTx_ApplicationEventCallback(pUSARTHandler, USART_EVENT_RX_CMPLT);
			}

		}
	}

	/**** Interrupt due to CTS flag *******************************/
	//Note: CTS is not applicable to UART4 and UART5

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_CTS)) && (pUSARTHandler->pUSARTx->CR3 & (1 << USART_CR3_CTSIE))
														  && (pUSARTHandler->pUSARTx->CR3 & (1 << USART_CR3_CTSE)))
	{
		//clear CTS flag
		pUSARTHandler->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//Notify the Application
		USARTx_ApplicationEventCallback(pUSARTHandler, USART_EVENT_CTS);
	}

	/**** Interrupt due to IDLE line Detection ********************/

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_IDLE)) && (pUSARTHandler->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE)))
	{
		__UNUSED uint32_t tmp;
		//clear IDLE Flag
		tmp = pUSARTHandler->pUSARTx->DR;

		//Notify the Application
		USARTx_ApplicationEventCallback(pUSARTHandler, USART_EVENT_IDLE);
	}

	/**** Interrupt due to OVR Error ******************************/

	if((pUSARTHandler->pUSARTx->SR & (1 << USART_SR_ORE)) && (pUSARTHandler->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE)))
	{
		//Notify the Application
		USARTx_ApplicationEventCallback(pUSARTHandler, USART_EVENT_ERR_ORE);
	}


}



