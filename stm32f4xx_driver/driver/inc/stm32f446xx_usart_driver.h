
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
 * stm32f446xx_usart_driver.h
 *
 *  Created on: 21-Jul-2020
 *      Author: Rajssss@GitHub.com
 *      Description: USART/UART Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Data structures and the address mapping for all USART/UART Peripherals
 *           - USART/UART configuration registers declarations and bits definition
 *           - Macros to access various USART/UART configuration Registers
 *           - Macros to various USART/UART configuration Registers Mode and options
 *           - Function prototypes to perform various operation on a USART/UART Bus
 *
 */



#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include  "stm32f446xx.h"

/*****************************************************************************************
 * USARTx Peripheral Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint8_t			USART_Mode;					//USART Peripheral MODE. @USART_MODE
	uint32_t		USART_Baud;					//USART Baud Rate. @USART_BAUD for standard baud rates
	uint8_t			USART_StopBits_No;			//USART Stop bits. @USART_STOPBITS
	uint8_t			USART_WordLength;			//USART Word Length. @USART_WORDLEN
	uint8_t			USART_ParityControl;		//USART Parity Control. @USART_PARITY
	uint8_t			USART_HWFlowControl;		//USART HW Flow Control. @USART_HWFLOW

}USARTx_Config_ty;



/*****************************************************************************************
 * USARTx Handler structure.
 *****************************************************************************************/
typedef struct
{
	USARTx_RegDef_ty *pUSARTx;
	USARTx_Config_ty USARTx_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t BusyTxState;					//USART Application State @USART_APP_STATS
	uint8_t BusyRxState;

}USARTx_Handler_ty;




/*****************************************************************************************
 * USARTx Mode configuration macros. @USART_MODE
 *****************************************************************************************/
#define 	USART_MODE_TX					0
#define 	USART_MODE_RX					1
#define 	USART_MODE_TXRX					2



/*****************************************************************************************
 * USARTx standard Baud rate configuration macros. @USART_BAUD
 *****************************************************************************************/
#define USART_BAUD_STD_1200					1200
#define USART_BAUD_STD_2400					2400
#define USART_BAUD_STD_9600					9600
#define USART_BAUD_STD_19200				19200
#define USART_BAUD_STD_38400 				38400
#define USART_BAUD_STD_57600 				57600
#define USART_BAUD_STD_115200 				115200
#define USART_BAUD_STD_230400 				230400
#define USART_BAUD_STD_460800 				460800
#define USART_BAUD_STD_921600 				921600
#define USART_BAUD_STD_2M 					2000000
#define USART_BAUD_STD_3M 					3000000



/*****************************************************************************************
 * USARTx Word Length configuration macros. @USART_WORDLEN
 *****************************************************************************************/
#define USART_WORD_LEN_8BITS				0
#define USART_WORD_LEN_9BITS				1



/*****************************************************************************************
 * USARTx Parity configuration macros. @USART_PARITY
 *****************************************************************************************/
#define USART_PARITY_DISABLE				0
#define USART_PARITY_EVEN					1
#define USART_PARITY_ODD					2



/*****************************************************************************************
 * USARTx Stop Bits configuration macros. @USART_STOPBITS
 *****************************************************************************************/
#define USART_STOP_BITS_1					0
#define USART_STOP_BITS_0_5					1
#define USART_STOP_BITS_2					2
#define USART_STOP_BITS_3					3



/*****************************************************************************************
 * USARTx Hardware Flow configuration macros. @USART_HWFLOW
 *****************************************************************************************/
#define USART_HW_FLOW_NONE					0
#define USART_HW_FLOW_CTS					1
#define USART_HW_FLOW_RTS					2
#define USART_HW_FLOW_CTS_RTS				3




/*****************************************************************************************
 * USARTx Status Flags macros for bit-masking. @USART_FLAG_BITS
 *****************************************************************************************/
#define USART_FLAG_PE						(1 << USART_SR_PE)
#define USART_FLAG_FE						(1 << USART_SR_FE)
#define USART_FLAG_NF						(1 << USART_SR_NF)
#define USART_FLAG_ORE						(1 << USART_SR_ORE)
#define USART_FLAG_IDLE						(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE						(1 << USART_SR_RXNE)
#define USART_FLAG_TC						(1 << USART_SR_TC)
#define USART_FLAG_TXE						(1 << USART_SR_TXE)
#define USART_FLAG_LBD						(1 << USART_SR_LBD)
#define USART_FLAG_CTS						(1 << USART_SR_CTS)



/*****************************************************************************************
 * USARTx Application State. @USART_APP_STATS
 *****************************************************************************************/
#define USART_READY							0
#define USART_BUSY_RX						1
#define USART_BUSY_TX						2



/*****************************************************************************************
 * USARTx Application Events
 *****************************************************************************************/
#define 	USART_EVENT_TX_CMPLT			0
#define 	USART_EVENT_RX_CMPLT			1
#define 	USART_EVENT_CTS					2
#define 	USART_EVENT_TC					3
#define 	USART_EVENT_IDLE				4
#define 	USART_EVENT_ERR_ORE				5
#define 	USART_EVENT_ERR_PE				6





/******************************************************************************************
 *								APIs supported by this driver
 *		     For more information about the APIs check the function definitions
 ******************************************************************************************/
//USART/UART Clock Control Handler
void USARTx_PeriClkControl(USARTx_RegDef_ty *pUSARTx, uint8_t Control);


//USART/UART Initialize/Deinitialize Handler
void USARTx_Init(USARTx_Handler_ty *pUSARTHandler);
void USARTx_DeInit(USARTx_RegDef_ty *pUSARTx);

//USART/UART Data Send and Receive handlers
void USARTx_SendData(USARTx_Handler_ty *pUSARTHandler, uint8_t *pTxBuffer, uint8_t length);
void USARTx_ReceiveData(USARTx_Handler_ty *pUSARTHandler, uint8_t *pRxBuffer, uint8_t length);
uint8_t USARTx_SendData_IT(USARTx_Handler_ty *pUSARTHandler, uint8_t *pTxBuffer, uint8_t length);
uint8_t USARTx_ReceiveData_IT(USARTx_Handler_ty *pUSARTHandler, uint8_t *pRxBuffer, uint8_t length);

//USART/UART IRQ and ISR Handlers
void USARTx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Control);
void USARTx_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USARTx_IRQHandling(USARTx_Handler_ty *pUSARTHandler);

//Other handlers
uint8_t USARTx_GetFlagStatus(USARTx_RegDef_ty *pUSARTx, uint32_t FlagName);
void USARTx_PeripheralControl(USARTx_RegDef_ty *pUSARTx, uint8_t Control);
void USARTx_ClearFlag(USARTx_RegDef_ty *pUSARTx, uint16_t FlagName);
void USARTx_BaudRate_Config(USARTx_RegDef_ty *pUSARTx, uint32_t BaudRate);


//Application Event Callback
__WEAK void USARTx_ApplicationEventCallback(USARTx_Handler_ty *pUSARTHandler, uint8_t AppEvent);





#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
