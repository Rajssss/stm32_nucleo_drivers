/*
 * 001LEDButton.c
 *
 *  Created on: 07-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: LED Control with User Button Control with polling Example for STM32F446RE-Nucleo Board.
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"



void delay()
{
	for(uint32_t i=0; i<50000/2; i++);
}

int main(void)
{
	GPIOx_Handler_ty GPIOLed, GPIOButton;

	//Configure the User LED, on GPIOA5
	GPIOx_PeriClkControl(GPIOA, ENABLE);

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = 5;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOLed);


	//Configure the User Button, on GPIOC13
	GPIOx_PeriClkControl(GPIOC, ENABLE);

	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = 13;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOButton);


	while(1)
	{
		if(!GPIOx_ReadFromInputPin(GPIOC, 13))				//if button status is high
		{
			delay();										//delay for button debouncing
			GPIOx_WriteToOutputPin(GPIOA, 5, HIGH);			//set GPIPOA5(LED) to HIGH
		}
		else
		{
			delay();										//delay for button debouncing
			GPIOx_WriteToOutputPin(GPIOA, 5, LOW);			//set GPIPOA5(LED) to LOW
		}
	}

	return 0;
}
