
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
 *           - Function definitions to perform various operation on a GPIO Port or Pin.
 *
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
 * @Note		-	1.FTSR: Bit 19 Reserved, must be kept at reset value.
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


		else												   				  			//3 Interrupt specific Modes
		{
			if(pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode == GPIO_MODE_IT_FT)			//Falling Edge Trigger
			{
				EXTI->RTSR &= ~(1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);	//clear corresponding RTSR bit
				EXTI->FTSR |= (1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);		//set corresponding FTSR bit
			}

			else if(pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode == GPIO_MODE_IT_RT)		//Rising Edge Trigger
			{
				EXTI->FTSR &= ~(1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);	//clear corresponding FTSR bit
				EXTI->RTSR |= (1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);		//set corresponding RTSR bit
			}

			else if(pGPIOHandler->GPIOx_PinConfig.GPIOx_PinMode == GPIO_MODE_IT_RFT)
			{
				EXTI->FTSR |= (1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);		//set both corresponding
				EXTI->RTSR |= (1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);		//RSTR and FTSR bit
			}

			//Enable Clock for SYSCFG
			SYSCFG_PCLK_EN();

			//Select and set the port for EXTI Line
			//1. PinNumber / 4 will select the required EXTICR Register
			//2. PinNumber % 4 will select the required bit range
			//3. (PinNumber % 4) * 4 will select the 4 bit set always
			//4. GPIO_BASEADDR_TO_CODE(PORT_BASSADDRESS) will generate a PORTCODE based on the base address of the port
			SYSCFG->EXTICR[pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber / 4] |= (GPIO_BASEADDR_TO_CODE(pGPIOHandler->pGPIOx) << ((pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber % 4) * 4));


			EXTI->IMR |= (1 << pGPIOHandler->GPIOx_PinConfig.GPIOx_PinNumber);			//0: Interrupt request from line x is masked
																						//1: Interrupt request from line x is not masked

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
 * @brief		-	This function Enable or Disable the given GPIO Port
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
 * @param[2]	-	Control: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void GPIOx_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Control)
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
 * 					 	GPIO Interrupt Priority Handler
 *
 * @fn: 		- 	GPIOx_IRQPriorityConfig
 *
 * @brief		-	This function configures the Interrupt Priority of a Port
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
void GPIOx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1.(IRQNumber / 4) will select the specific PR resister out of 60 PR registers then
	//3.(NVIC_PR_BASE_ADDR + (IRQNumber / 4) will finally jumps to the required base address of the PR register
	//4.(IRQNumber % 4) will select the specific bit section for a PR register.
	//5.((IRQNumber % 4) * 8) will select the 8 bit section range.
	//6.(8 - NVIC_NO_PR_BITS_IMPLEMENTED) will selct the required addition shifts, as NVIC_NO_PR_BITS_IMPLEMENTED is vendor specific
	//7.(((IRQNumber % 4) * 8) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED)) will finally give the required shift amount.

	*(NVIC_PR_BASE_ADDR + (IRQNumber / 4)) |= (IRQPriority << (((IRQNumber % 4) * 8) + (8 - NVIC_NO_PR_BITS_IMPLEMENTED)));

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
	//clear the EXTI Register bit corresponding to the Pin Number.

	if(EXTI->PR & (1 << PinNumber))	//confirm the the intrust is pending EXTI
	{
		EXTI->PR |= (1 << PinNumber); //clear from EXTI PR
	}
}






