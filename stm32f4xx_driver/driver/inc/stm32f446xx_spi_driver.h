
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

#include "stm32f446xx.h"


/*****************************************************************************************
 * SPIx Peripheral Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint8_t SPIx_DeviceMode;						//Possible Device Modes from @SPI_MODES
	uint8_t	SPIx_BusConfig;							//Possible Bus Configurations from @SPI_BUS_CONFIGS
	uint8_t SPIx_SCLKSpeed;							//Possible SCLK Speeds from @SPI_SCLK_CONFIGS
	uint8_t SPIx_DFF;								//Possible DFF Configurations from @SPI_DFF_CONFIGS
	uint8_t SPIx_CPOL;								//Possible CPOL Configurations from @SPI_CPOL_CONFIGS
	uint8_t SPIx_CPHA;								//Possible CPHA Configurations from @SPI_CPHA_CONFIGS
	uint8_t SPIx_SSM;								//Possible SSM Configurations from @SPI_SSM_CONFIGS

}SPIx_Config_ty;



/*****************************************************************************************
 * SPIx Handler structure.
 *****************************************************************************************/
typedef struct
{
	SPIx_RegDef_ty 		*pSPIx;							/*!<Holds the base address of the SPI Peripheral to which the Configuration belongs.>*/
	SPIx_Config_ty 		SPIx_Config;					/*!<Holds SPI configurations settings.>*/
	uint8_t				*pTxBuffer;
	uint8_t				*pRxBuffer;
	uint32_t			TxLength;
	uint32_t			RxLength;
	uint8_t				TxState;
	uint8_t				RxState;

}SPIx_Handler_ty;






/*****************************************************************************************
 * SPIx Possible Device Modes Macros. @SPI_MODES
 *****************************************************************************************/
#define 	SPI_DEVICE_MODE_MASTER					1
#define 	SPI_DEVICE_MODE_SLAVE					0



/*****************************************************************************************
 * SPIx Possible Bus Configuration macros. @SPI_BUS_CONFIGS
 *****************************************************************************************/
#define 	SPI_BUS_CONFIG_FD						1
#define 	SPI_BUS_CONFIG_HD						2
#define 	SPI_BUS_CONFIG_RXONLY					3




/*****************************************************************************************
 * SPIx Possible Bus Configuration macros. @SPI_SCLK_CONFIGS
 *****************************************************************************************/
#define 	SPI_SCLK_DIV2							0
#define 	SPI_SCLK_DIV4							1
#define 	SPI_SCLK_DIV8							2
#define 	SPI_SCLK_DIV16							3
#define 	SPI_SCLK_DIV32							4
#define 	SPI_SCLK_DIV64							5
#define 	SPI_SCLK_DIV128							6
#define 	SPI_SCLK_DIV256							7





/*****************************************************************************************
 * SPIx Status Flags macros for bit-masking. @SPI_FLAG_BITS
 *****************************************************************************************/
#define 	SPI_FLAG_RXNE							(0 << SPI_SR_RXNE)
#define 	SPI_FLAG_TXE							(1 << SPI_SR_TXE)
#define 	SPI_FLAG_CHSIDE							(2 << SPI_SR_CHSIDE)
#define 	SPI_FLAG_UDR							(3 << SPI_SR_UDR)
#define 	SPI_FLAG_CRCERR							(4 << SPI_SR_CRCERR)
#define 	SPI_FLAG_MODF							(5 << SPI_SR_MODF)
#define 	SPI_FLAG_OVR							(6 << SPI_SR_OVR)
#define 	SPI_FLAG_BSY							(7 << SPI_SR_BSY)
#define 	SPI_FLAG_FRE							(8 << SPI_SR_FRE)






/*****************************************************************************************
 * SPIx Possible DFF Configuration macros. @SPI_DFF_CONFIGS
 *****************************************************************************************/
#define 	SPI_DFF_8BIT							0
#define 	SPI_DFF_16BIT							1




/*****************************************************************************************
 * SPIx Possible CPOL Configuration macros. @SPI_CPOL_CONFIGS
 *****************************************************************************************/
#define 	SPI_CPOL_LOW							0
#define 	SPI_CPOL_HIGH							1



/*****************************************************************************************
 * SPIx Possible CPHA Configuration macros. @SPI_CPHA_CONFIGS
 *****************************************************************************************/
#define 	SPI_CPHA_LOW							0
#define 	SPI_CPHA_HIGH							1



/*****************************************************************************************
 * SPIx Possible SSM Configuration macros. @SPI_SSM_CONFIGS
 *****************************************************************************************/
#define 	SPI_SSM_DI								0
#define 	SPI_SSM_EN								1




/*****************************************************************************************
 * SPIx Application States
 *****************************************************************************************/
#define		SPI_READY		0
#define 	SPI_BUSY_RX		1
#define 	SPI_BUSY_TX		2



/*****************************************************************************************
 * SPIx Application Events
 *****************************************************************************************/
#define 	SPI_EVENT_TX_CMPLT		1
#define 	SPI_EVENT_RX_CMPLT		2
#define 	SPI_EVENT_ERR_OVR		3
#define 	SPI_EVENT_ERR_CRC		4



/*****************************************************************************************
 * 							APIs Supported by this Driver
 * 				For more information about the APIs check definition.
*****************************************************************************************/
//GPIO Clock Control Handler
void SPIx_PeriClkControl(SPIx_RegDef_ty *pSPIx, uint8_t Control);

//SPI Initialize/Deinitialize Handler
void SPIx_Init(SPIx_Handler_ty *pSPIHandler);
void SPIx_DeInit(SPIx_RegDef_ty *pSPIx);


//SPI Data Send and Receive handlers
uint8_t SPIx_SendDataIT(SPIx_Handler_ty *pSPIhandler, uint8_t *pTxBuffer, uint32_t Length);		//Interrupt based transmission
void SPIx_SendData(SPIx_RegDef_ty *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPIx_ReceiveDataIT(SPIx_Handler_ty *pSPIhandler, uint8_t *pRxBuffer, uint32_t Length);	//Interrupt based reception
void SPIx_ReceiveData(SPIx_RegDef_ty *pSPIx, uint8_t *pRxBuffer, uint32_t Length);



//SPI IRQ and ISR Handlers
void SPIx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Control);
void SPIx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPIx_IRQHandling(SPIx_Handler_ty *pSPIHandler);

//Other handlers
uint8_t SPIx_GetFlagStatus(SPIx_RegDef_ty *pSPIx, uint32_t Flag);	//Read the Status Register of SPIx
void SPIx_PeriControl(SPIx_RegDef_ty *pSPIx, uint8_t Control);		//Enable or Disable the SPIx Peripheral
void SPIx_SSOEConfig(SPIx_RegDef_ty *pSPIx, uint8_t Control);		//Enable or Disable the SPOE of SPIx Peripheral
void SPIx_SSIConfig(SPIx_RegDef_ty *pSPIx, uint8_t Control);		//Enable or Disable the NSS of SPIx Peripheral
void SPIx_Abort_Tx(SPIx_Handler_ty *pSPIHandler);					//Abort SPI Transmission
void SPIx_Abort_Rx(SPIx_Handler_ty *pSPIHandler);					//Abort SPI Reception


//Application Event Callback
__WEAK void SPIx_ApplicationEventCallback(SPIx_Handler_ty *pSPIHandler, uint8_t AppEvent);		//User Application need to implement it as per need


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
