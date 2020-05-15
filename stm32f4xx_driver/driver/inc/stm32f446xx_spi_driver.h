
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
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 15-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: SPI Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Data structures and the address mapping for all SPI Peripherals
 *           - SPI configuration registers declarations and bits definition
 *           - Macros to access various SPI configuration Registers
 *           - Macros to various SPI configuration Registers Mode and options
 *           - Function prototypes to perform various operation on a SPI Bus
 *
 */


#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32f446xx.h"

/*****************************************************************************************
 * GPIOx Pin Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint8_t SPIx_DeviceMode;
	uint8_t	SPIx_BusConfig;
	uint8_t SPIx_SCLKSpeed;
	uint8_t SPIx_DFF;
	uint8_t SPIx_CPOL;
	uint8_t SPIx_CPHL;
	uint8_t SPIx_SSM;

}SPIx_Config_ty;



/*****************************************************************************************
 * GPIOx Handler structure.
 *****************************************************************************************/
typedef struct
{
	SPIx_RegDef_ty *pSPIx;							/*!<Holds the base address of the SPI Peripheral to which the Configuration belongs.>*/
	SPIx_Config_ty GPIOx_PinConfig;					/*!<Holds SPI configurations settings.>*/

}SPIx_Handler_ty;




/*****************************************************************************************
 * 							APIs Supported by this Driver
 * 				For more information about the APIs check definition.
 *****************************************************************************************/
//SPI Initialize/Deinitialize Handler
void SPIx_Init(SPIx_Handler_ty *pSPIHandler);
void SPIx_DeInit(SPIx_RegDef_ty *pSPIx);


//SPI Data Send and Receive handlers
void SPIx_SendData(SPIx_RegDef_ty *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPIx_ReceiveData(SPIx_RegDef_ty *pSPIx, uint8_t *pRxBuffer, uint32_t Length);



//SPI IRQ and ISR Handlers
void SPIx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Control);
void SPIx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPIx_IRQHandling(SPIx_Handler_ty *pSPIHandler);





#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
