/*
 * 003LEDBtnInterrupt.c
 *
 *  Created on: May 11, 2020
 *      Author: Raj.S
 */


#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"



void delay()
{
	for(uint32_t i=0; i<50000/2; i++);
}

int main(void)
{
	GPIOx_Handler_ty GPIOLed, GPIOButton;

	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOButton, 0, sizeof(GPIOButton));


	//Configure the User LED, on GPIOA5

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOLed);


	//Configure the User Button, on GPIOC13
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOButton);



	//IRQ Configuration
	GPIOx_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1);


	return 0;
}



//ISR
void EXTI15_10_IRQHandler(void)
{
	//Warning: its an terrible idea to put a delay inside an ISR, but for the sake of simplicity
	delay();

	GPIOx_IRQHandling(GPIO_PIN_NO_13);

	//Toggle the USER LED
	GPIOx_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

}


