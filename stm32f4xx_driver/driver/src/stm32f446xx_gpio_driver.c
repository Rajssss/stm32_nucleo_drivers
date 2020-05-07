
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
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 05-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: GPIO Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Function definations to perform various operation on a GPIO Port or Pin.
 *
 *		TODO: Interrupt handlers.
 */

#include <stm32f446xx_gpio_driver.h>



/***********************************************************************************
 * 					 	GPIO Initialize Handler
 *
 * @fn: 		- 	GPIOx_Init
 *
 * @brief		-	This function will configure the various Registers associated with that
 * 					GPIO Port with the values present in GPIOx_PinConfig structure.
 *
 * @param[1]	-	Address of the Handler Request.
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_Init(GPIOx_Handler_ty *pGPIOHandler)
{
	{//Configure the mode of GPIOx Pin. Refer @GPIO_MODE
		if(pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode <= GPIO_MODE_ANALOG)   //4 general GPIO pin Modes
		{
			pGPIOHandler->pGPIOx->MODER &= ~(3 << (2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber))); //Clear the Mode Bits first

			pGPIOHandler->pGPIOx->MODER |= pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode << ((2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber))); //Now set the mode
		}


		else												   				  //3 Interrupt specific Modes
		{

		}
	}


	{//Configure the mode of GPIOx Pin. Refer @GPIO_OSPEED
		pGPIOHandler->pGPIOx->OSPEEDER &= ~(3 << (2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber))); //Clear the bits first

		pGPIOHandler->pGPIOx->OSPEEDER |= (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinSpeed) << (2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber)); //Now Set the speed
	}


	{//Configure the mode of GPIOx Pin. Refer @GPIO_PUPD
		pGPIOHandler->pGPIOx->PUPDR &= ~(3 << (2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber))); 	//Clear the bits first

		pGPIOHandler->pGPIOx->PUPDR |= (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinPuPdControl) << (2 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber));
	}


	{//Configure the mode of GPIOx Pin. Refer @GPIO_OTYPE
		pGPIOHandler->pGPIOx->OTYPER |= pGPIOHandler->GPIOx_PinConfig.GPIOx_PinOPType << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber;
	}


	{//Configure the mode of GPIOx Pin. Refer @GPIO_ALTFN
		if(pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode == GPIO_MODE_ALTFN)  //Alternate function is only meaningful when GPIO mode is set to Alternative Function
		{
			/******************************************************************
			 * -> PinNo/8 selects the HIGH or LOW Alternate Function Registers.
			 * -> if PinNo/8 = 0, the LOW Alternative function Register will be selected (AFR[0]).
			 * -> if PinNo/8 = , the HIGH Alternative function Register will be selected (AFR[1]).
			 *
			 * ->PinNo%8 selects bit-set for a Pin in HIGH or LOW register.
			 * ->Example: if PinNo%8 = 1, 1st bit-set of the AFR will be selected.
			 * */

			pGPIOHandler->pGPIOx->AFR[pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber / 8] &= ~(0xF << (4 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber % 8))); //clear the bits first

			pGPIOHandler->pGPIOx->AFR[pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber / 8] |= (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinAltFunMode << (4 * (pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber % 8))); //Now set the bits
		}
	}

}


/***********************************************************************************
 * 					 	GPIO Deinitialize Handler
 *
 * @fn: 		- 	GPIOx_Deinit
 *
 * @brief		-	This function resets all the registers associated with that GPIO Port,
 * 					thus Deinitialize the given GPIO Port.
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_DeInit(GPIOx_RegDef_ty *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}



/***********************************************************************************
 * 					 	GPIO Clock Control Handler
 *
 * @fn: 		- 	GPIOx_PeriClkControl
 *
 * @brief		-	This function Deinitialize the given GPIO Port
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_PeriClkControl(GPIOx_RegDef_ty *pGPIOx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DE();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DE();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DE();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DE();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DE();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DE();
		}

	}

}





/***********************************************************************************
 * 					 	GPIOx Pin Read Handler
 *
 * @fn: 		- 	GPIOx_ReadFromInputPin
 *
 * @brief		-	This function Reads data from the given GPIOx Pin
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @param[2]	-	Pin Number (0 to 15)
 *
 * @return		-	0 or 1 (uint_8)
 *
 * @Note		-	none
 *
 */
uint8_t GPIOx_ReadFromInputPin(GPIOx_RegDef_ty *pGPIOx, uint8_t PinNumber)
{
	return ((uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001));  //shift the whole register bits till LSB, then extract LSB.
}


/***********************************************************************************
 * 					 	GPIO Port Read Handler
 *
 * @fn: 		- 	GPIOx_ReadFromInputPort
 *
 * @brief		-	This function Reads 16-bit data from the given GPIO Port
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @return		-	uint16_t
 *
 * @Note		-	none
 *
 */
uint16_t GPIOx_ReadFromInputPort(GPIOx_RegDef_ty *pGPIOx)
{
	return ((uint16_t) (pGPIOx->IDR));

}




/***********************************************************************************
 * 					 	GPIO Pin Write Handler
 *
 * @fn: 		- 	GPIOx_WriteToOutputPin
 *
 * @brief		-	This function Writes given data to the given GPIOx Pin
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @param[2]	-	Pin Number of the Port
 *
 * @param[3]	-	Value to be written: HIGH or LOW
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_WriteToOutputPin(GPIOx_RegDef_ty *pGPIOx,uint8_t PinNumber, uint8_t value)
{
	if(value == 1)
	{
		pGPIOx->ODR |= (1 << PinNumber);			//Set the corresponding bit
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);			//Reset the corresponding bit
	}

}


/***********************************************************************************
 * 					 	GPIO Port Write Handler
 *
 * @fn: 		- 	GPIOx_WriteToOutputPort
 *
 * @brief		-	This function Writes given 16-bit data to the given GPIO Port
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @param[2]	-	Value to be written: HIGH or LOW
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_WriteToOutputPort(GPIOx_RegDef_ty *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;							//copy full 16bit value to ODR.
}


/***********************************************************************************
 * 					 	GPIO Toggle Output Pin Handler
 *
 * @fn: 		- 	GPIOx_ToggleOutputPin
 *
 * @brief		-	This function Writes given 16-bit data to the given GPIO Port
 *
 * @param[1]	-	Base Address of the GPIO Port
 *
 * @param[2]	-	Value to be written: HIGH or LOW
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_ToggleOutputPin(GPIOx_RegDef_ty *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);				//Toggle the corresponding bit in ODR.
}




/***********************************************************************************
 * 					 	GPIO Interrupt Configuration-Handler
 *
 * @fn: 		- 	GPIOx_IRQConfig
 *
 * @brief		-	This function configures the Interrupt functionality of a Port
 *
 * @param[1]	-	IRQ number of the Interrupt
 *
 * @param[2]	-	Interrupt Priority
 *
 * @param[3]	-	Control: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Control)
{

}


/***********************************************************************************
 * 					 	GPIO Interrupt Handler
 *
 * @fn: 		- 	GPIOx_IRQHandling
 *
 * @brief		-	Interrupt handler of A GPIO Port
 *
 * @param[1]	-	Pin number of the Port
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_IRQHandling(uint8_t PinNumber)
{

}






