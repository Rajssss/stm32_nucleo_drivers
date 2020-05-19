/*
 * 004SPI_Tx_Test.c
 *
 *  Created on: 18-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: SPI Data Transmit test program for STM32F446RE-Nucleo Board.
 */

#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"


void SPI2_GPIOInit(void);			//to initialize GPIO pins for SPI2
void SPI2_Init(void);				//to initialize SPI2 Registers


// PB12 --> 			SPI2_NSS
// PB13 --> 			SPI2_SCLK
// PB14 --> 			SPI2_MISO
// PB15 --> 			SPI2_MOSI
// ALT Function --> 	5



int main()
{
	char user_data[] = "Hello World!";

	SPI2_GPIOInit();					//Initialize GPIOB pins (12..15) for SPI2 use.

	SPI2_Init();						//Initialize SPI2 Peripheral Registers

	SPIx_SSIConfig(SPI2, ENABLE);		//this will make the NSS High and avoid MODF error (muti-master)

	SPIx_PeriControl(SPI2, ENABLE);		//enable the SIP2 peripheral

	SPIx_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));			//send the user_data

	SPIx_PeriControl(SPI2, DISABLE);		//Disable the SIP2 peripheral after transmission complete


	while(1);

}



void SPI2_GPIOInit(void)
{
	GPIOx_Handler_ty SPI2_Pins;

	SPI2_Pins.pGPIOx = GPIOB;						//use port-B

	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_ALTFN;
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 5;
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOx_Init(&SPI2_Pins);

	//MOSI
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_15;
	GPIOx_Init(&SPI2_Pins);

	/*
	//MISO									//not necessary as no actual slave is used in this test
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_14;
	GPIOx_Init(&SPI2_Pins);

	//NSS									//not necessary as no actual slave is used in this test
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_12;
	GPIOx_Init(&SPI2_Pins);
	*/

}


void SPI2_Init()
{
	SPIx_Handler_ty SPI2_Handler;

	SPI2_Handler.pSPIx = SPI2;							//use SPI2
	SPI2_Handler.SPIx_Config.SPIx_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handler.SPIx_Config.SPIx_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handler.SPIx_Config.SPIx_DFF = SPI_DFF_8BIT;
	SPI2_Handler.SPIx_Config.SPIx_SCLKSpeed = SPI_SCLK_DIV2;	//8MHz SCLK
	SPI2_Handler.SPIx_Config.SPIx_CPHA = SPI_CPHA_LOW;
	SPI2_Handler.SPIx_Config.SPIx_CPOL = SPI_CPOL_LOW;
	SPI2_Handler.SPIx_Config.SPIx_SSM = SPI_SSM_EN;				//not necessary as no actual slave is used in this test

	SPIx_Init(&SPI2_Handler);

}

