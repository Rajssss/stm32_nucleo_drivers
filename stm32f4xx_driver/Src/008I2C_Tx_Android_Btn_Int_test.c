/*
 * 007I2C_Tx_Arduino_test.c
 *
 *  Created on: 04-Jul-2020
 *      Author: Raj.S
 *      Description: I2C Data Transmit example program with button interrupt
 *      for STM32F446RE-Nucleo Board with Arduino Slave.
 *
 */

//includes
#include <string.h>
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_i2c_driver.h"


//slave address
#define SLAVE_ADDR	0x68

/* PB10 -> I2C2_SCL
 * PB3 -> I2C2_SDA
 */

//prototypes
void I2C2_GPIOInit(void);			//GPIO init for I2C1 Pins
void I2C2_Init(void);				//I2C1 Peripheral init
void delay(void);					//Delay Function
void GPIO_LEDInit(void);			//User LED
void GPIO_ButtonInit(void);			//User Button



void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}



//Objects
I2Cx_Handler_ty I2C2Handler;
GPIOx_Handler_ty I2C2Pins, GPIOLed, GPIOButton;


//User Data
uint8_t user_data[] = "Hello Embedded World!\n";



int main(void)
{

 	memset(&I2C2Handler, 0, sizeof(I2C2Handler));
	memset(&I2C2Pins, 0, sizeof(I2C2Pins));
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOButton, 0, sizeof(GPIOButton));


	//Initialize GPIO Pins to work with I2C2
	I2C2_GPIOInit();

	//Initialize User LED
	GPIO_LEDInit();

	//Initialize User Button
	GPIO_ButtonInit();

	//Initialize I2C2
	I2C2_Init();

	//Interrupt Priority
	GPIOx_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);

	//Enable Interrupt
	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;
}



void I2C2_GPIOInit(void)
{
	I2C2Pins.pGPIOx = GPIOB;
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_ALTFN;
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinAltFunMode = 4;
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_OD;		//In I2C, GPIO pins must be in Open Drain.
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_PU;		//In our test case, Internal is used, but in practical you have to calculate it and use external one.
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_10;
	GPIOx_Init(&I2C2Pins);

	//SDA
	I2C2Pins.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_3;
	GPIOx_Init(&I2C2Pins);

}



void I2C2_Init(void)
{
	I2C2Handler.pI2Cx = I2C2;
	I2C2Handler.I2Cx_Config.I2C_ACKControl = I2C_ACK_ENABLE;

	/*Not applicable in master mode*/
	I2C2Handler.I2Cx_Config.I2C_DeviceAddrMode = I2C_DeviceAddr_7BIT;
	I2C2Handler.I2Cx_Config.I2C_DualDeviceAddrConfig = I2C_DeviceAddr_Single;
	I2C2Handler.I2Cx_Config.I2C_DeviceAddress_Primary = 0x61;

	//Not applicable in Standard Mode
	I2C2Handler.I2Cx_Config.I2C_FMDutyCycle = I2C_DUTY_2;

	I2C2Handler.I2Cx_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD;

	I2Cx_Init(&I2C2Handler);


}


//ISR
void EXTI15_10_IRQHandler(void)
{
	//Warning: its a terrible idea to put a delay inside an ISR, but for the sake of simplicity
	delay();

	GPIOx_IRQHandling(GPIO_PIN_NO_13);

	//Toggle the USER LED
	GPIOx_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

	//Initiate Data transfer
	I2Cx_SendData_Master(&I2C2Handler, user_data, strlen((char *)user_data), SLAVE_ADDR, I2C_SR_ENABLE);
}


void GPIO_LEDInit(void)
{
	//Configure the User LED, on GPIOA5

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOLed.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOLed);
}



void GPIO_ButtonInit(void)
{
	//Configure the User Button, on GPIOC13
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;

	GPIOx_Init(&GPIOButton);
}
