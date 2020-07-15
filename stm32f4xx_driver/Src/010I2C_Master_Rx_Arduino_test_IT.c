/*
 * 009I2C_Master_Rx_Arduino_test.c
 *
 *  Created on: 09-Jul-2020
 *      Author: Raj.S
 *      Description: Interrupt based I2C Master Data Receive example program for STM32F446RE-Nucleo
 *      Board with Arduino Slave. This example Command Based communication.
 *      This example uses interrupt to detect button-pressed.
 *
 */

//includes
#include "stm32f446xx.h"
#include "stdio.h"


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


//semihosting
//extern void initialise_monitor_handles(void);


//objects
I2Cx_Handler_ty I2C1Handler;
GPIOx_Handler_ty I2C1Pins, GPIOLED, GPIOButton;


//Received Data Buffer
uint8_t Received_Data_Buffer[32];


int main(void)
{

	//initialise_monitor_handles();
	//printf("Semihosting begin...\n\r");

	I2C1_GPIOInit();

	GPIO_LEDInit();

	GPIO_ButtonInit();

	I2C1_Init();

	I2Cx_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);

	I2Cx_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	GPIOx_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_12);

	GPIOx_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	//printf("All inits done.. \n\r");

	while(1);

	return 0;

}



void delay()
{
	for(uint32_t i=0; i<500000/8; i++);
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

	//printf("begin SEND_LENGTH.. \n\r");

	//Send SEND_LENGTH command
	while(I2Cx_SendData_MasterIT(&I2C1Handler, &CommandCode, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY );

	//printf("end SEND_LENGTH.. \n\r");

	//printf("begin receive length.. \n\r");

	//Receive "length" of the data
	while(I2Cx_ReceiveData_MasterIT(&I2C1Handler, &Received_Data_Length, 1, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);

	//printf("end receive length.. \n\r");

	CommandCode = CMD_SEND_DATA;

	//printf("begin SEND_DATA.. \n\r");

	//Send SEND_DATA command
	while(I2Cx_SendData_MasterIT(&I2C1Handler, &CommandCode, sizeof(CommandCode), SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);

	//printf("begin receive data.. \n\r");

	//Receive Data
	while(I2Cx_ReceiveData_MasterIT(&I2C1Handler, Received_Data_Buffer, Received_Data_Length, SLAVE_ADDR, I2C_SR_ENABLE) != I2C_READY);

	//printf("end receivedata.. \n\r");


	Received_Data_Buffer[Received_Data_Length + 1] = '\0';

	//printf("received data=>%s\n\r", Received_Data_Buffer);


}



void I2Cx_ApplicationEventCallback(I2Cx_Handler_ty *pI2CHandler, uint8_t AppEvent)
{
	//do whatever you want for an event or error
	//Examples

	if(AppEvent == I2C_EV_TX_CMPLT)
	{

	}
	else if(AppEvent == I2C_EV_RX_CMPLT)
	{

	}
	else if(AppEvent == I2C_EV_STOP)
	{

	}
	else if(AppEvent == I2C_ER_AF)
	{
		//ACK failure

		//so close communication
		I2Cx_Close_INTRSendData(pI2CHandler);

		//Gartered STOP

		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//hang
		while(1);
	}
}



//ISRs
void EXTI15_10_IRQHandler()
{
	//Button Debouncing (you should not put any delay inside any ISR. nut just for the sake of simplicity)
	delay();

	//printf("Button pressed.. \n\r");

	GPIOx_IRQHandling(GPIO_PIN_NO_13);

	//Toggle the User-LED
	GPIOx_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

	//Begin I2C Communication
	I2C1_Begin();
}



void I2C1_EV_IRQHandler(void)
{
	I2Cx_EV_IRQHandling(&I2C1Handler);
}



void I2C1_ER_IRQHandler(void)
{
	I2Cx_ER_IRQHandling(&I2C1Handler);
}
