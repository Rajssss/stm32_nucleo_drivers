/*
 * 004SPI_Tx_Test.c
 *
 *  Created on: 18-May-2020
 *      Author: Rajssss@GitHub.com
 *      Description: SPI Command and Response based communication for
 *      STM32F446RE-Nucleo Board with Arduino Slave.
 */

#include <string.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

//Function Prototypes
void SPI2_GPIOInit(void);			//to initialize GPIO pins for SPI2
void SPI2_Init(void);				//to initialize SPI2 Registers
void delay(void);					//Delay Function
void GPIO_ButtonInit(void);			//to initialize User Button at GPIOC13
uint8_t SPI2_VerifyResponse(uint8_t);		//verify ACK or NACK

extern void initialise_monitor_handles();			//Semihosting


// PB12 --> 			SPI2_NSS
// PB13 --> 			SPI2_SCLK
// PB14 --> 			SPI2_MISO
// PB15 --> 			SPI2_MOSI
// ALT Function --> 	5


//Command Codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_ANALOG_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54



#define LED_ON			1
#define LED_OFF			0


//Arduino Slave Pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

#define DIGITAL_PIN9		9
#define DIGITAL_PIN8		8


void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}



int main()
{
	uint8_t CommandCode;
	uint8_t Dummy_Write = 0xFF;
	uint8_t Dummy_Read;
	uint8_t Ack_Response_Byte;
	uint8_t Args[2];

	initialise_monitor_handles();		//Semihosting

	printf("Application Started!\n");

	SPI2_GPIOInit();					//Initialize GPIOB pins (12..15) for SPI2 use.

	GPIO_ButtonInit();

	SPI2_Init();						//Initialize SPI2 Peripheral Registers

	printf("Init Done!\n");


	/*
	 * IF Slave Select Output Enable (SSOE) is set then, NSS O/P will depend on SPE
	 * IF SSM=0 and SPI Peri is Enabled (SPE), the NSS will be pulled 0 automatically
	 * IF SSM=0 and SPI Peri is Disabled (SPE), the NSS will be set 1 automatically
	 */
	SPIx_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//1.LED CTRL
		while( GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();							//Button De-bounce

		SPIx_PeriControl(SPI2, ENABLE);		//enable the SIP2 peripheral

		CommandCode = COMMAND_LED_CTRL;
		SPIx_SendData(SPI2, &CommandCode, 1);			//send 1 Byte led command code (0x50)

		SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

		SPIx_SendData(SPI2, &Dummy_Write, 1);			//send a dummy byte to get receive 1 Byte

		SPIx_ReceiveData(SPI2, &Ack_Response_Byte, 1);		//Receive ACK

		if( SPI2_VerifyResponse(Ack_Response_Byte) )			//if ACK
		{
			//send LED command args
			Args[0] = DIGITAL_PIN9;
			Args[1]	= LED_ON;
			SPIx_SendData(SPI2, Args, 2);			//send LED Command Args

			printf("Led On!\n");


		}
		//2.Analog Read
		while(GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		CommandCode = COMMAND_ANALOG_READ;
		SPIx_SendData(SPI2, &CommandCode, 1);			//send 1 Byte led command code (0x50)

		SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

		SPIx_SendData(SPI2, &Dummy_Write, 1);			//send a dummy byte to get receive 1 Byte

		SPIx_ReceiveData(SPI2, &Ack_Response_Byte, 1);		//Receive ACK

		if( SPI2_VerifyResponse(Ack_Response_Byte) )			//if ACK
		{
			//send LED command args
			Args[0] = ANALOG_PIN0;
			SPIx_SendData(SPI2, Args, 1);			//send LED Command Args

			SPIx_ReceiveData(SPI2, &Dummy_Read, 1);		//Dummy Read to clear RXNE

			delay();

			SPIx_SendData(SPI2, &Dummy_Write, 1);		//Dummy write

			uint8_t Analog_Data;
			SPIx_ReceiveData(SPI2, &Analog_Data, 1);	//Receive Analog Pin data

			printf("Analog Pin val = %d\n", Analog_Data);


		}


		//3.LED Status
		while(GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		CommandCode = COMMAND_LED_READ;
		SPIx_SendData(SPI2, &CommandCode, 1);			//send 1 Byte led command code (0x50)

		SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

		SPIx_SendData(SPI2, &Dummy_Write, 1);			//send a dummy byte to get receive 1 Byte

		SPIx_ReceiveData(SPI2, &Ack_Response_Byte, 1);		//Receive ACK

		if( SPI2_VerifyResponse(Ack_Response_Byte) )			//if ACK
		{
			//send command args
			Args[0] = DIGITAL_PIN9;
			SPIx_SendData(SPI2, Args, 1);			//send LED Command Args

			SPIx_ReceiveData(SPI2, &Dummy_Read, 1);		//Dummy Read to clear RXNE

			delay();

			SPIx_SendData(SPI2, &Dummy_Write, 1);		//Dummy write

			uint8_t Led_Status;
			SPIx_ReceiveData(SPI2, &Led_Status, 1);	//Receive Analog Pin data

			printf("Led Status = %d\n", Led_Status);

		}


		//4.CMD_Print (Send a string to Slave)
		while(GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		CommandCode = COMMAND_PRINT;
		SPIx_SendData(SPI2, &CommandCode, 1);			//send 1 Byte command code

		SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

		SPIx_SendData(SPI2, &Dummy_Write, 1);			//send a dummy byte to get receive 1 Byte

		SPIx_ReceiveData(SPI2, &Ack_Response_Byte, 1);		//Receive ACK

		uint8_t msg[] = "Hello Embedded World!";
		if( SPI2_VerifyResponse(Ack_Response_Byte) )			//if ACK
		{
			Args[0] = strlen((char *)msg);

			SPIx_SendData(SPI2, Args, 1);			//send length of msg

			SPIx_ReceiveData(SPI2, &Dummy_Read, 1);		//Dummy Read to clear RXNE

			SPIx_SendData(SPI2, msg, Args[0]);		//send the msg

			SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

			printf("String Sent!\n");
		}


		//5.Get Slave ID
		while(GPIOx_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		CommandCode = COMMAND_ID_READ;
		SPIx_SendData(SPI2, &CommandCode, 1);			//send 1 Byte command code

		SPIx_ReceiveData(SPI2, &Dummy_Read, 1);			//Dummy Read Byte to clear RXNE

		SPIx_SendData(SPI2, &Dummy_Write, 1);			//send a dummy byte to get receive 1 Byte

		SPIx_ReceiveData(SPI2, &Ack_Response_Byte, 1);		//Receive ACK

		uint8_t id[11];

		if( SPI2_VerifyResponse(Ack_Response_Byte) )			//if ACK
		{
			for (int var = 0; var < 10; var++)
			{
				SPIx_SendData(SPI2, &Dummy_Write, 1);
				SPIx_ReceiveData(SPI2, &id[var], 1);
			}

			id[11] = '\0';

			printf("String Recieved = %s\n", id);
		}




	}
	SPIx_PeriControl(SPI2, DISABLE);

			printf("Com closed!\n");
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
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_PU;
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOx_Init(&SPI2_Pins);

	//MOSI
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_15;
	GPIOx_Init(&SPI2_Pins);


	//MISO
	SPI2_Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_14;
	GPIOx_Init(&SPI2_Pins);


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



uint8_t SPI2_VerifyResponse(uint8_t Ack_Response_Byte)
{
	if(Ack_Response_Byte == 0xF5)
	{
		//ACK
		return 1;
	}
	else
	{
		//NACK
		return 0;

	}
}
