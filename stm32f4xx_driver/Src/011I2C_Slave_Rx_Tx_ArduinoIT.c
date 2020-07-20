/*
 * 011I2C_Slave_Rx_Tx_ArduinoIT.c
 *
 *  Created on: 17-Jul-2020
 *      Author: Raj.S
 *
 *      Description: Interrupt based I2C Slave Rx/Tx example program for STM32F446RE-Nucleo
 *      Board with Arduino Arduino. This example uses Command Based communication.
 *
 */

//includes
#include "stm32f446xx.h"
#include <string.h>


//Slave Address
#define MY_ADDR			0x68


//Prototypes
void I2C1_GPIOInit(void);
void I2C1_Init(void);



//objects
I2Cx_Handler_ty I2C1Handler;
GPIOx_Handler_ty I2C1Pins;


int main(void)
{
	I2C1_GPIOInit();

	I2C1_Init();

	I2Cx_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);

	I2Cx_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2Cx_CallbackEV_SlaveControl(I2C1, ENABLE);

	while(1);

}



void I2C1_GPIOInit()
{
	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_ALTFN;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 4;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_OD;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	//PB6 -> SCL
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_6;
	GPIOx_Init(&I2C1Pins);

	//P7 -> SDA
	I2C1Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_7;
	GPIOx_Init(&I2C1Pins);
}


void I2C1_Init()
{
	I2C1Handler.pI2Cx = I2C1;
	I2C1Handler.I2Cx_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handler.I2Cx_Config.I2C_DeviceAddrMode = I2C_DeviceAddr_7BIT;
	I2C1Handler.I2Cx_Config.I2C_DualDeviceAddrConfig = I2C_DeviceAddr_Single;
	I2C1Handler.I2Cx_Config.I2C_DeviceAddress_Primary = MY_ADDR;
	I2C1Handler.I2Cx_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD;
	I2Cx_Init(&I2C1Handler);
}


void I2Cx_ApplicationEventCallback(I2Cx_Handler_ty *pI2CHandler, uint8_t AppEvent)
{
	static uint8_t Tx_Data_Buffer[] = "Raj.S!!";
	static uint8_t CommandCode = 0, count = 0;

	if(AppEvent == I2C_EV_DATA_REQ)
	{
		//Send data to master
		if(CommandCode == 0x51)
		{
			//send data length
			I2Cx_SendData_Slave(pI2CHandler->pI2Cx, strlen((char *)Tx_Data_Buffer));
		}
		else if(CommandCode == 0x52)
		{
			//send data
			I2Cx_SendData_Slave(pI2CHandler->pI2Cx, Tx_Data_Buffer[count++]);
		}

	}
	else if(AppEvent == I2C_EV_DATA_RCV)
	{
		//Read data from master
		CommandCode = I2Cx_ReceiveData_Slave(pI2CHandler->pI2Cx);
	}
	else if(AppEvent == I2C_ER_AF)
	{
		//Master nest NACK, do not send more data
		//reset parameters
		CommandCode = 0;
		count = 0;

	}
	else if(AppEvent == I2C_EV_STOP)
	{
		//Master ended communication with slave

	}
}



void I2C1_EV_IRQHandler(void)
{
	I2Cx_EV_IRQHandling(&I2C1Handler);
}



void I2C1_ER_IRQHandler(void)
{
	I2Cx_ER_IRQHandling(&I2C1Handler);
}


