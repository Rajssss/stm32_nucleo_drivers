
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
 *           - Function definitions to perform various operation on a GPIO Port or Pin.
 *
 *
 */


#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"




/***********************************************************************************
 * 					 	GPIO Clock Control Handler
 *
 * @fn: 		- 	GPIOx_PeriClkControl
 *
 * @brief		-	This function Deinitialize the given GPIO Port
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
 * @Note		-	TODO: SPI Interrupts
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
 * @Note		-	TODO: Complete DeInitialize
 *
 */
void SPIx_DeInit(SPIx_RegDef_ty *pSPIx)
{

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
 * 					TODO: I hate polling, so make it interrupt based.
 *
 */
void SPIx_SendData(SPIx_RegDef_ty *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{
	while(Length > 0)
	{
		//wait till TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		//check DFF in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16-bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);		//covert to 16-bit then deref to DR
			Length -= 2;								//Decrease Tx Length 1 Bytes
			(uint16_t *)pTxBuffer++;					//Increase Buffer by 1
		}
		else
		{
			//8-bit DFF
			pSPIx->DR = *pTxBuffer;						//covert to 16-bit then deref to DR
			Length--;									//Decrease Tx Length 1 Bytes
			pTxBuffer++;								//Increase Buffer by 1
		}

	}

}


/***********************************************************************************
 * 					 	Child function of SPIx_SendData
 *
 * @fn: 		- 	SPI_GetFlagStatus
 *
 * @brief		-	Child function of SPIx_SendData which returns status of a flag
 * 					of Status Register (SR).
 *
 * @param[1]	-	Base Address of the SPI Peripheral
 * @param[2]	-	Flag value from @SPI_FLAG_BITS
 *
 * @return		-	uint8_t
 *
 * @Note		-
 *
 */
uint8_t SPI_GetFlagStatus(SPIx_RegDef_ty *pSPIx, uint32_t Flag)
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
 * @Note		-
 *
 */
void SPIx_ReceiveData(SPIx_RegDef_ty *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{

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




