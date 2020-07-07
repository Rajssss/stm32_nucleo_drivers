/*
 * 009I2C_Master_Rx_Arduino_test.c
 *
 *  Created on: 06-Jul-2020
 *      Author: Raj.S
 *      Description: I2C Master Data Receive example program for STM32F446RE-Nucleo
 *      Board with Arduino Slave. This example Command Based communication.
 *      This example uses interrupt to detect button-pressed.
 *
 */

//includes
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_i2c_driver.h"


//Command Codes
#define CMD_SEND_DATA		0x52
#define CMD_SEND_LENGTH		0x51


//Slave Address
#define SLAVE_ADDR			0x68


//Prototypes
void delay(void);
void I2C1_GPIOInit(void);
void GPIO_LEDInit(void);
void GPIO_ButtonInit(void);
void I2C1_Init(void);
void I2C1_Begin(void);



//objects
I2Cx_Handler_ty I2C1Handler;
GPIOx_Handler_ty I2C1Pins, GPIOLED, GPIOButton;


//Received Data Buffer
uint8_t Received_Data_Buffer[32];


int main(void)
{
	I2C1_GPIOInit();

	GPIO_LEDInit();

	GPIO_ButtonInit();

	I2C1_Init();

	GPIOx_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_12);

	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;

}



void delay()
{
	for(uint32_t i=0; i<500000/4; i++);
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




void GPIO_LEDInit()
{
	GPIOLED.pGPIOx = GPIOA;
	GPIOLED.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIOx_PinConfig.GPIOx_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOLED.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;
	GPIOLED.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_5;
	GPIOx_Init(&GPIOLED);
}




void GPIO_ButtonInit()
{
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinMode = GPIO_MODE_IT_FT;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIOx_PinConfig.GPIOx_PinNumber = GPIO_PIN_NO_13;
	GPIOx_Init(&GPIOButton);
}




void I2C1_Init()
{
	I2C1Handler.pI2Cx = I2C1;
	I2C1Handler.I2Cx_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handler.I2Cx_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STD;
	I2Cx_Init(&I2C1Handler);
}



//I2C Communication Handler
void I2C1_Begin()
{
	uint8_t CommandCode = CMD_SEND_LENGTH;

	uint8_t Received_Data_Length;			///To store Length of data which is to be received from slave

	//Send SEND_LENGTH command
	I2Cx_SendData_Master(&I2C1Handler, &CommandCode, sizeof(CommandCode), SLAVE_ADDR, I2C_SR_ENABLE);

	//Receive "length" of the data
	I2Cx_ReceiveData_Master(&I2C1Handler, &Received_Data_Length, 1, SLAVE_ADDR, I2C_SR_ENABLE);

	CommandCode = CMD_SEND_DATA;

	//Send SEND_DATA command
	I2Cx_SendData_Master(&I2C1Handler, &CommandCode, sizeof(CommandCode), SLAVE_ADDR, I2C_SR_ENABLE);

	//Receive Data
	I2Cx_ReceiveData_Master(&I2C1Handler, Received_Data_Buffer, Received_Data_Length, SLAVE_ADDR, I2C_SR_ENABLE);

	Received_Data_Buffer[Received_Data_Length + 1] = '\0';

}



void EXTI15_10_IRQHandler()
{
	//Button Debouncing (you should not put any delay inside any ISR. nut just for the sake of simplicity)
	delay();

	GPIOx_IRQHandling(GPIO_PIN_NO_13);

	//Toggle the User-LED
	GPIOx_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

	//Begin I2C Communication
	I2C1_Begin();
}

