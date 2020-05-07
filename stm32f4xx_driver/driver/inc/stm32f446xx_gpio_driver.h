
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
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 05-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: GPIO Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Data structures and the address mapping for all GPIO
 *           - GPIO configuration registers declarations and bits definition
 *           - Macros to access various GPIO configuration Registers
 *           - Macros to various GPIO configuration Registers Mode and options
 *           - Function prototypes to perform various operation on a GPIO Port or Pin.
 *
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>



/*****************************************************************************************
 * @GPIO_MODE
 * GPIOx Pin Supported Modes
 *****************************************************************************************/
#define 	GPIO_MODE_IN		0
#define 	GPIO_MODE_OUT		1
#define 	GPIO_MODE_ALTFN		2
#define 	GPIO_MODE_ANALOG	3
//Interrupts related Modes
#define 	GPIO_MODE_IT_FT		4
#define 	GPIO_MODE_IT_RT		5
#define 	GPIO_MODE_IT_RFT	6




/*****************************************************************************************
 * @GPIO_OTYPE
 * GPIOx Pin Output Types
 *****************************************************************************************/
#define 	GPIO_OP_TYPE_PP		0
#define 	GPIO_OP_TYPE_OD		1




/*****************************************************************************************
 * @GPIO_OSPEED
 * GPIOx Pin Output Speeds
 *****************************************************************************************/
#define 	GPIO_SPEED_LOW		0
#define 	GPIO_SPEED_MID		1
#define 	GPIO_SPEED_FAST		2
#define 	GPIO_SPEED_HIGH		3




/*****************************************************************************************
 * @GPIO_PUPD
 * GPIOx Pin Pull-Up & Pull-Down Configurations
 *****************************************************************************************/
#define 	GPIO_PIN_NO_PUPD	0
#define 	GPIO_PIN_PU			1
#define 	GPIO_PIN_PD			2




/*****************************************************************************************
 * GPIOx Pin Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint8_t GPIOx_PinNumber;					//Possible values are from 0 to 15
	uint8_t GPIOx_PinSpeed;						//Possible values from @GPIO_OSPEED
	uint8_t GPIOx_PinMode;						//Possible values from @GPIO_MODE
	uint8_t GPIOx_PinPuPdControl;				//Possible values from @GPIO_PUPD
	uint8_t GPIOx_PinOPType;					//Possible values from @GPIO_OTYPE
	uint8_t GPIOx_PinAltFunMode;

}GPIO_PinConfig_ty;






/*****************************************************************************************
 * GPIOx Handler structure.
 *****************************************************************************************/
typedef struct
{
	GPIOx_RegDef_ty *pGPIOx;							/*!<Holds the base address of the GPIO Port to which the Pin belongs.>*/
	GPIO_PinConfig_ty GPIOx_PinConfig;					/*!<Holds pin configurations settings.>*/

}GPIOx_Handler_ty;






/*****************************************************************************************
 * 							APIs Supported by this Driver
 * 				For more information about the APIs check definition.
 *****************************************************************************************/
//GPIO Initialize/Deinitialize Handler
void GPIOx_Init(GPIOx_Handler_ty *pGPIOHandler);
void GPIOx_DeInit(GPIOx_RegDef_ty *pGPIOx);



//GPIO Clock Control Handler
void GPIOx_PeriClkControl(GPIOx_RegDef_ty *pGPIOx, uint8_t Control);



//GPIO Read/Write Handlers
uint8_t GPIOx_ReadFromInputPin(GPIOx_RegDef_ty *pGPIOx, uint8_t PinNumber);
uint16_t GPIOx_ReadFromInputPort(GPIOx_RegDef_ty *pGPIOx);
void GPIOx_WriteToOutputPin(GPIOx_RegDef_ty *pGPIOx,uint8_t PinNumber, uint8_t value);
void GPIOx_WriteToOutputPort(GPIOx_RegDef_ty *pGPIOx, uint16_t value);
void GPIOx_ToggleOutputPin(GPIOx_RegDef_ty *GPIOx, uint8_t PinNumber);



//GPIO IRQ Handlers
void GPIOx_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Control);
void GPIOx_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
