/*
 * 004SPI_Tx_Test.c
 *
 *  Created on: 18-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: SPI Data Transmit example program for STM32F446RE-Nucleo Board
 *      			 with Arduino Slave.
 */

#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"


void SPI2_GPIOInit(void);			//to initialize GPIO pins for SPI2
void SPI2_Init(void);				//to initialize SPI2 Registers
void delay(void);					//Delay Function
void GPIO_ButtonInit(void);			//to initialize User Button at GPIOC13


// PB12 --> 			SPI2_NSS
// PB13 --> 			SPI2_SCLK
// PB14 --> 			SPI2_MISO
// PB15 --> 			SPI2_MOSI
// ALT Function --> 	5


void delay()
{
	for(uint32_t i=0; i<50000/2; i++);
}



int main()
{
	char user_data[] = "Hello World!";

	SPI2_GPIOInit();					//Initialize GPIOB pins (12..15) for SPI2 use.

	GPIO_ButtonInit();

	SPI2_Init();						//Initialize SPI2 Peripheral Registers

	/*
	 * IF Slave Select Output Enable (SSOE) is set then, NSS O/P will depend on SPE
	 * IF SSM=0 and SPI Peri is Enabled (SPE), the NSS will be pulled 0 automatically
	 * IF SSM=0 and SPI Peri is Disabled (SPE), the NSS will be set 1 automatically
	 */
	SPIx_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(! GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			delay();

			SPIx_PeriControl(SPI2, ENABLE);		//enable the SIP2 peripheral

			uint8_t dataLen = strlen(user_data);

			SPIx_SendData(SPI2, &dataLen, 1);			//send length of user_data first

			SPIx_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));			//send the user_data

			SPIx_PeriControl(SPI2, DISABLE);		//Disable the SIP2 peripheral after transmission complete
		}
	}

}



void GPIO_ButtonInit()
{
	GPIOx_Handler_ty GPIOButton;

	//Configure the User Button, on GPIOC13
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOButton);

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
	*/

	//NSS									//Hardware Slave Management
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_12;
	GPIOx_Init(&SPI2_Pins);

}


void SPI2_Init()
{
	SPIx_Handler_ty SPI2_Handler;

	SPI2_Handler.pSPIx = SPI2;												//use SPI2
	SPI2_Handler.SPIx_Config.SPIx_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handler.SPIx_Config.SPIx_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handler.SPIx_Config.SPIx_DFF = SPI_DFF_8BIT;
	SPI2_Handler.SPIx_Config.SPIx_SCLKSpeed = SPI_SCLK_DIV8;				//2MHz SCLK
	SPI2_Handler.SPIx_Config.SPIx_CPHA = SPI_CPHA_LOW;
	SPI2_Handler.SPIx_Config.SPIx_CPOL = SPI_CPOL_LOW;
	SPI2_Handler.SPIx_Config.SPIx_SSM = SPI_SSM_DI;							//Hardware Slave management

	SPIx_Init(&SPI2_Handler);

}
