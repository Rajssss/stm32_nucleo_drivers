/*
 * 001LEDToggle.c
 *
 *  Created on: 07-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: LED Toggle Example for STM32F446RE-Nucleo Board.
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"



void delay()
{
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	GPIOx_Handler_ty GPIOLed;

	GPIOx_PeriClkControl(GPIOA, ENABLE);

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = 5;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOLed);

	while(1)
	{
		//GPIOx_ToggleOutputPin(GPIOA, 5);
		GPIOx_WriteToOutputPin(GPIOA, 5, HIGH);
		delay();
		GPIOx_WriteToOutputPin(GPIOA, 5, LOW);
		delay();
	}

	return 0;
}
