
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
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 16-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: SPI Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Function definitions to perform various operation on a SPI Peripheral.
 *
 *
 */


#include "stm32f446xx_spi_driver.h"


static void SPIx_OVR_ERR_INTR_Hanlder(SPIx_Handler_ty *pSPIHandler);
static void SPIx_RXE_INTR_Hanlder(SPIx_Handler_ty *pSPIHandler);
static void SPIx_TXE_INTR_Handler(SPIx_Handler_ty *pSPIHandler);


/***********************************************************************************
 * 					 	SPI Clock Control Handler
 *
 * @fn: 		- 	SPIx_PeriClkControl
 *
 * @brief		-	This function Deinitialize the given SPI Port
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void SPIx_PeriClkControl(SPIx_RegDef_ty *pSPIx, uint8_t Control)
{
	if(Control == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}

		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}

		}

}





/***********************************************************************************
 * 					 	SPI Initialize Handler
 *
 * @fn: 		- 	SPIx_Init
 *
 * @brief		-	This function will configure the various Registers associated with that
 * 					SPI Peripheral with the values present in SPIx_Config structure.
 *
 * @param[1]	-	Address of the SPI Handler Request.
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_Init(SPIx_Handler_ty *pSPIHandler)
{
	//Enable Peripheral Clock for SPIx first
	SPIx_PeriClkControl(pSPIHandler->pSPIx, ENABLE);

	{//1st Program SPI CR1 Register

		//Device Mode (Master/Slave)
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_DeviceMode << SPI_CR1_MSTR);

		{//Bus Config (Full Duplex/Half Duplex/Simplex)
			if(pSPIHandler->SPIx_Config.SPIx_BusConfig == SPI_BUS_CONFIG_FD)
			{
				pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
			}
			else if(pSPIHandler->SPIx_Config.SPIx_BusConfig == SPI_BUS_CONFIG_HD)
			{
				pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
			}
			else if(pSPIHandler->SPIx_Config.SPIx_BusConfig == SPI_BUS_CONFIG_RXONLY)
			{
				pSPIHandler->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);	//clear BIDIMODE first
				pSPIHandler->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);	//set RXONLYMODE
			}
		}


		//CPHA
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_CPHA << SPI_CR1_CPHA);

		//CPOL
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_CPOL << SPI_CR1_CPOL);

		//SCLK Speed
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_SCLKSpeed << SPI_CR1_BR);

		//DFF
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_DFF << SPI_CR1_DFF);

		//SSM
		pSPIHandler->pSPIx->CR1 |= (pSPIHandler->SPIx_Config.SPIx_SSM << SPI_CR1_SSM);

	}


}




/***********************************************************************************
 * 					 	SPI Deinitialize Handler
 *
 * @fn: 		- 	SPIx_DeInit
 *
 * @brief		-	This function resets all the registers associated with that SPI Peripheral,
 * 					thus Deinitialize the given SPI Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_DeInit(SPIx_RegDef_ty *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PERI_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_PERI_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PERI_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_PERI_RESET();
	}

}





/***********************************************************************************
 * 					 	SPI Data Transmit Handler with Interrupt
 *
 * @fn: 		- 	SPIx_SendDataIT
 *
 * @brief		-	This function confusers vacuous requirements of data transmission
 * 					of SPIx Peripheral. This is an Interrupt based Transmission.
 *
 * @param[1]	-	Pointer SPI handler
 * @param[2]	-	Transmit Buffer
 * @param[3]	-	Length of the data to be transmitted
 *
 * @return		-	uint8_t
 *
 * @Note		-	This function returns the Transmission state of SPI(Busy/Free)
 *
 */
uint8_t SPIx_SendDataIT(SPIx_Handler_ty *pSPIhandler, uint8_t *pTxBuffer, uint32_t Length)
{
	if(pSPIhandler->TxState != SPI_BUSY_TX)
	{
		//copy TxBuffer info to Handler
		pSPIhandler->pTxBuffer = pTxBuffer;
		pSPIhandler->TxLength = Length;

		//Mark SPIx as busy, so no other program can take it over
		pSPIhandler->TxState = SPI_BUSY_TX;

		//Enable TXEIE in CR2
		pSPIhandler->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//Finally data transmission will be handled by ISR

	}

	return (pSPIhandler->TxState);

}





/***********************************************************************************
 * 					 	SPI Data Transmit Handler
 *
 * @fn: 		- 	SPIx_SendData
 *
 * @brief		-	This function writes the data to be transmitted to Data Register (DR)
 * 					of SPIx Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Transmit Buffer
 * @param[3]	-	Length of the data to be transmitted
 *
 * @return		-	void
 *
 * @Note		-	This is a blocking call.
 *
 */
void SPIx_SendData(SPIx_RegDef_ty *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		//wait till TXE is set
		while(SPIx_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		//check DFF in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16-bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);		//covert to 16-bit then dereference to DR
			Length -= 2;								//Decrease Tx Length 1 Bytes
			(uint16_t *)pTxBuffer++;					//Increase Buffer by 1
		}
		else
		{
			//8-bit DFF
			pSPIx->DR = *pTxBuffer;
			Length--;									//Decrease Tx Length 1 Bytes
			pTxBuffer++;								//Increase Buffer by 1
		}

	}

}






/***********************************************************************************
 * 					 	SPI Data Receive Handler with Interrupt
 *
 * @fn: 		- 	SPIx_ReceiveDataIT
 *
 * @brief		-	This function confusers vacuous requirements of data reception
 * 					of SPIx Peripheral. This is an Interrupt based Reception.
 *
 * @param[1]	-	Pointer to SPI handler
 * @param[2]	-	Receive Buffer
 * @param[3]	-	Length of the data to be receive
 *
 * @return		-	uint8_t
 *
 * @Note		-	This function returns the Transmission state of SPI(Busy/Free)
 *
 */
uint8_t SPIx_ReceiveDataIT(SPIx_Handler_ty *pSPIhandler, uint8_t *pRxBuffer, uint32_t Length)
{
	if(pSPIhandler->RxState != SPI_BUSY_RX)
	{
		//copy TxBuffer info to Handler
		pSPIhandler->pRxBuffer = pRxBuffer;
		pSPIhandler->RxLength = Length;

		//Mark SPIx as busy, so no other program can take it over
		pSPIhandler->RxState = SPI_BUSY_RX;

		//Enable TXEIE in CR2
		pSPIhandler->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//Finally data receive will be handled by ISR

	}

	return (pSPIhandler->RxState);

}




/***********************************************************************************
 * 					 	SPI Data Receive Handler
 *
 * @fn: 		- 	SPIx_ReceiveData
 *
 * @brief		-	This function reads the data being received to Data Register (DR)
 * 					of SPIx Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Receive Buffer
 * @param[3]	-	Length of the data to be receive
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_ReceiveData(SPIx_RegDef_ty *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		while(SPIx_GetFlagStatus(pSPIx, SPI_FLAG_RXNE));

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16-bit
			*((uint16_t *) pRxBuffer) = pSPIx->DR;
			Length -= 2;
			(uint16_t *)pRxBuffer++;

		}
		else
		{
			//8-bit
			(*(uint8_t *) pRxBuffer)=pSPIx->DR;
			Length--;
			(uint8_t *)pRxBuffer++;

		}
	}

}





/***********************************************************************************
 * 					 		SPI Flag Status Handler
 *
 * @fn: 		- 	SPIx_GetFlagStatus
 *
 * @brief		-	Function which returns status of a flag of Status Register (SR).
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Flag value from @SPI_FLAG_BITS
 *
 * @return		-	uint8_t
 *
 * @Note		-
 *
 */
uint8_t SPIx_GetFlagStatus(SPIx_RegDef_ty *pSPIx, uint32_t Flag)
{
	if(pSPIx->SR & Flag)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}




/***********************************************************************************
 * 					 	SPIx Peripheral Control Handler
 *
 * @fn: 		- 	SPI_PeriControl
 *
 * @brief		-	This function Enable or Disable the SPIx Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Control value (Enable or Disable)
 *
 * @return		-	void
 *
 * @Note		-	SPI peripherals must be kept disable in initialization and only
 * 					enable before sending or receiving data.
 *
 */
void SPIx_PeriControl(SPIx_RegDef_ty *pSPIx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		while(SPIx_GetFlagStatus(SPI2, SPI_FLAG_BSY));					//wait till SPI2 is not Busy
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}





/***********************************************************************************
 * 					 	SPIx Peripheral NSS Config Handler
 *
 * @fn: 		- 	SPIx_SSIConfig
 *
 * @brief		-	This function Enable or Disable NSS of SPIx Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Control value (Enable or Disable)
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_SSIConfig(SPIx_RegDef_ty *pSPIx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}




/***********************************************************************************
 * 					 	SPIx Peripheral SSOE Config Handler
 *
 * @fn: 		- 	SPIx_SSOEConfig
 *
 * @brief		-	This function Enable or Disable SSOE of SPIx Peripheral.
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Control value (Enable or Disable)
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_SSOEConfig(SPIx_RegDef_ty *pSPIx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}





/***********************************************************************************
 * 					 	SPIx Peripheral Interrupt Handler
 *
 * @fn: 		- 	SPIx_SSOEConfig
 *
 * @brief		-	This function configures Interrupt Requests configurations of SPIx Peripheral.
 *
 * @param[1]	-	pointer to the Handler of SPI Peripheral
 *
 * @return		-	void
 *
 * @Note		-	There are 6 checks (TXE, RXNE, MODF, OVR, CRCERR. FRE) out of which
 * 					3 (TXE, RXNE, OVR) are implemented
 *
 */
void SPIx_IRQHandling(SPIx_Handler_ty *pSPIHandler)
{
	//check if TXE and and TXEIE is set
	if((pSPIHandler->pSPIx->SR & (1 << SPI_SR_TXE)) &&
				(pSPIHandler->pSPIx->CR2 & (1 << SPI_CR2_TXEIE)))
	{
		//TXE Handler
		SPIx_TXE_INTR_Handler(pSPIHandler);
	}

	//check if RXNE and RXNEIE is set
	if((pSPIHandler->pSPIx->SR & (1 << SPI_SR_RXNE)) &&
			(pSPIHandler->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE)))
	{
		//RXE Handler
		SPIx_RXE_INTR_Hanlder(pSPIHandler);
	}

	//check if RXNE and RXNEIE is set
	if((pSPIHandler->pSPIx->SR & (1 << SPI_SR_OVR)) &&
			(pSPIHandler->pSPIx->CR2 & (1 << SPI_CR2_ERRIE)))
	{
		////OVR Error Handler
		SPIx_OVR_ERR_INTR_Hanlder(pSPIHandler);
	}

}




/***********************************************************************************
 * 					 	SPI Data Transmit Interrupt Handler
 *
 * @fn: 		- 	SPIx_TXE_INTR_Handler
 *
 * @brief		-	This function writes the data to be transmitted to Data Register (DR)
 * 					of SPIx Peripheral.This is a helper function of SPIx_SendData.
 *
 * @param[1]	-	Pointer SPI handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
static void SPIx_TXE_INTR_Handler(SPIx_Handler_ty *pSPIHandler)
{
	//check DFF in CR1
	if(pSPIHandler->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16-bit Transmission
		pSPIHandler->pSPIx->DR = *((uint16_t *) pSPIHandler->pTxBuffer);
		pSPIHandler->TxLength -= 2;
		(uint16_t *)pSPIHandler->pTxBuffer++;
	}
	else
	{
		//8-bit Transmission
		pSPIHandler->pSPIx->DR = *((uint8_t *) pSPIHandler->pTxBuffer);
		pSPIHandler->TxLength--;
		pSPIHandler->pTxBuffer++;
	}

	if(! (pSPIHandler->TxLength))
	{
		//close transmission
		SPIx_Abort_Tx(pSPIHandler);
		//inform the application
		SPIx_ApplicationEventCallback(pSPIHandler, SPI_EVENT_TX_CMPLT);
	}

}



/***********************************************************************************
 * 					 	SPI Data Receive Interrupt Handler
 *
 * @fn: 		- 	SPIx_RXE_INTR_Hanlder
 *
 * @brief		-	This function reads the data being received to Data Register (DR)
 * 					of SPIx Peripheral.This is a helper function of SPIx_ReceiveData.
 *
 * @param[1]	-	Pointer to SPI handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
static void SPIx_RXE_INTR_Hanlder(SPIx_Handler_ty *pSPIHandler)
{
	//check DFF in CR1
	if(pSPIHandler->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16-bit Transmission
		pSPIHandler->pSPIx->DR = *((uint16_t *) pSPIHandler->pTxBuffer);
		pSPIHandler->TxLength -= 2;
		(uint16_t *)pSPIHandler->pTxBuffer++;
	}
	else
	{
		//8-bit Transmission
		pSPIHandler->pSPIx->DR = *((uint8_t *) pSPIHandler->pTxBuffer);
		pSPIHandler->TxLength--;
		pSPIHandler->pTxBuffer++;
	}

	if(! (pSPIHandler->RxLength))
	{
		//close Reception
		SPIx_Abort_Rx(pSPIHandler);
		//inform the application
		SPIx_ApplicationEventCallback(pSPIHandler, SPI_EVENT_RX_CMPLT);
	}

}




/***********************************************************************************
 * 					 	SPI OVR Error Interrupt Handler
 *
 * @fn: 		- 	SPIx_OVR_ERR_INTR_Hanlder
 *
 * @brief		-	This function handles the Overrun Error Interrupt of
 * 					SPIx Peripheral.
 *
 * @param[1]	-	Pointer to SPI handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
static void SPIx_OVR_ERR_INTR_Hanlder(SPIx_Handler_ty *pSPIHandler)
{
	uint8_t __UNUSED temp;
	//clear OVR Flag
	if(pSPIHandler->TxState != SPI_BUSY_TX)
	{
		temp = pSPIHandler->pSPIx->DR;
		temp = pSPIHandler->pSPIx->SR;
	}

	//Inform Application about OVR
	SPIx_ApplicationEventCallback(pSPIHandler, SPI_EVENT_ERR_OVR);

}




/***********************************************************************************
 * 					 	SPI Transmission Abort Handler
 *
 * @fn: 		- 	SPIx_Abort_Tx
 *
 * @brief		-	This function aborts the ongoing transmission of data in
 * 					SPIx Peripheral.
 *
 * @param[1]	-	Pointer to SPI handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_Abort_Tx(SPIx_Handler_ty *pSPIHandler)
{
	pSPIHandler->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);		//disable TXE interrupt
	pSPIHandler->pTxBuffer = NULL;							//Nullify the transmission buffer
	pSPIHandler->TxLength = 0;								//set length of data to zero
	pSPIHandler->TxState = SPI_READY;						//set SPIx status as READY
}




/***********************************************************************************
 * 					 	SPI Reception Abort Handler
 *
 * @fn: 		- 	SPIx_Abort_Rx
 *
 * @brief		-	This function aborts the ongoing reception of data in
 * 					SPIx Peripheral.
 *
 * @param[1]	-	Pointer to SPI handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_Abort_Rx(SPIx_Handler_ty *pSPIHandler)
{
	pSPIHandler->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);		//disable RXNE interrupt
	pSPIHandler->pRxBuffer = NULL;							//Nullify the Receiver buffer
	pSPIHandler->RxLength = 0;								//set length of data to zero
	pSPIHandler->RxState = SPI_READY;						//set SPIx status as READY
}





/***********************************************************************************
 * 					 	SPI Clear Overrun Flag Handler
 *
 * @fn: 		- 	SPIx_ClearOVRFlag
 *
 * @brief		-	This function clears the overrun flag of SPIx Peripheral.
 *
 * @param[1]	-	Base Address of SPI Peripheral
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void SPIx_ClearOVRFlag(SPIx_RegDef_ty *pSPIx)
{
	uint8_t __UNUSED temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
}




/***********************************************************************************
 * 					 	SPI Application Even Callback Handler
 *
 * @fn: 		- 	SPIx_ApplicationEventCallback
 *
 * @brief		-	This function handles the callback after an application event
 * 					is occurred.
 *
 * @param[1]	-	Pointer to SPI Handler
 *
 * @param[2]	-	Application Event
 *
 * @return		-	void
 *
 * @Note		-	This is a weak implementation, and should be implemented in user
 * 					application
 *
 */

__WEAK void SPIx_ApplicationEventCallback(SPIx_Handler_ty *pSPIHandler, uint8_t AppEvent)
{
	//This is a weak implementation
}






