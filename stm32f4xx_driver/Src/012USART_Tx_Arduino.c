/*
 * 012USART_Tx_Arduino.c
 *
 *  Created on: 26-Jul-2020
 *      Author: Raj.S
 *      Description: USART (Async) transmission example program for STM32F446RE-Nucleo
 *      			 Board with Arduino.
 *
 */

//includes
#include "stm32f446xx.h"
#include <string.h>

//prototypes
static void delay(void);
static void USART2_GPIOInit(void);
static void USART2_Init(void);
static void GPIO_ButtonInit(void);

//user data to be transmitted
char user_data[1024] = "Raj.S\n";

//objects
USARTx_Handler_ty USART2_Handler;
GPIOx_Handler_ty GPIOButton, USART2_GPIO;



int main(void)
{
	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();

	while(1)
	{
		while(! GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			//Button Debouncing
			delay();

			USARTx_SendData(&USART2_Handler, (uint8_t*)user_data, strlen(user_data));
		}

	}
	return 0;
}


//delay for button debouncing
static void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


static void GPIO_ButtonInit()
{
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOx_Init(&GPIOButton);
}


static void USART2_Init()
{
	USART2_Handler.pUSARTx = USART1;
	USART2_Handler.USARTx_Config.USART_Baud = USART_BAUD_STD_115200;
	USART2_Handler.USARTx_Config.USART_HWFlowControl = USART_HW_FLOW_NONE;
	USART2_Handler.USARTx_Config.USART_Mode = USART_MODE_TX;
	USART2_Handler.USARTx_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2_Handler.USARTx_Config.USART_StopBits_No = USART_STOP_BITS_1;
	USART2_Handler.USARTx_Config.USART_WordLength = USART_WORD_LEN_8BITS;
	USARTx_Init(&USART2_Handler);
}


static void USART2_GPIOInit()
{
	USART2_GPIO.pGPIOx = GPIOB;
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_ALTFN;
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinAltFunMode = 7;
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_PU;
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	//TX
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_6;
	GPIOx_Init(&USART2_GPIO);

	//RX
	USART2_GPIO.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_7;
	GPIOx_Init(&USART2_GPIO);
}

