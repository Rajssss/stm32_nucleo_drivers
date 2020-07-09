
/***********************************************************************************
 * This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ***********************************************************************************/

/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 16-June-2020
 *      Author: Rajssss@GitHub.com
 *      Description: I2C Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Function definitions to perform various operation on I2C Peripheral.
 *
 *
 */

//includes
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_rcc_driver.h"



//Private Handlers
static void I2Cx_ClearADDRFlag(I2Cx_Handler_ty *pI2CHandler);
static void I2Cx_MasterHandle_RXNEInterrupt(I2Cx_Handler_ty *pI2CHandler);
static void I2Cx_MasterHandle_TXEInterrupt(I2Cx_Handler_ty *pI2CHandler);



/***********************************************************************************
 * 					 	I2C Clock Control Handler
 *
 * @fn: 		- 	I2Cx_PeriClkControl
 *
 * @brief		-	This function Deinitialize the given I2C Peripheral
 *
 * @param[1]	-	Base Address of the I2C Peripheral
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void I2Cx_PeriClkControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control)
{
	if(Control == ENABLE)
			{
				if(pI2Cx == I2C1)
				{
					I2C1_PCLK_EN();
				}
				else if(pI2Cx == I2C2)
				{
					I2C2_PCLK_EN();
				}
				else if(pI2Cx == I2C3)
				{
					I2C3_PCLK_EN();
				}

			}
			else
			{
				if(pI2Cx == I2C1)
				{
					I2C1_PCLK_DI();
				}
				else if(pI2Cx == I2C2)
				{
					I2C2_PCLK_DI();
				}
				else if(pI2Cx == I2C3)
				{
					I2C3_PCLK_DI();
				}

			}


}





/***********************************************************************************
 * 					 	I2C peripheral Control Handler
 *
 * @fn: 		- 	I2Cx_PeripheralControl
 *
 * @brief		-	This function Enable or Disable the given I2C Peripheral
 *
 * @param[1]	-	Base Address of the I2C Peripheral
 *
 * @param[2]	-	Control value: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-	none
 *
 */
void I2Cx_PeripheralControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control)
{
	if(Control == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}





/***********************************************************************************
 * 					 	  	 I2C Initialize Handler
 *
 * @fn: 		- 	I2Cx_Init
 *
 * @brief		-	This function will configure the various Registers associated with that
 * 					I2C Peripheral with the values present in I2Cx_Config structure.
 *
 * @param[1]	-	Address of the I2C Handler Request.
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void I2Cx_Init(I2Cx_Handler_ty *pI2CHandler)
{
	//Enable Peripheral Clock for I2Cx
	I2Cx_PeriClkControl(pI2CHandler->pI2Cx, ENABLE);

	//Enable Peripheral
	I2Cx_PeripheralControl(pI2CHandler->pI2Cx, ENABLE);

	//ACK Control
	pI2CHandler->pI2Cx->CR1 |= (pI2CHandler->I2Cx_Config.I2C_ACKControl << I2C_CR1_ACK);

	//Frequency
	 pI2CHandler->pI2Cx->CR2 |= ((RCC_GetPeriCLK1_Value() / 1000000U) & 0x3F);


	 //Primary Device Address
	 if(pI2CHandler->I2Cx_Config.I2C_DeviceAddrMode == I2C_DeviceAddr_10BIT)
	 {
		 pI2CHandler->pI2Cx->OAR1 |= ((0x3 << 14)  + pI2CHandler->I2Cx_Config.I2C_DeviceAddress_Primary);
	 }
	 else
	 {
		 pI2CHandler->pI2Cx->OAR1 |= ((0x1 << 14)  + (pI2CHandler->I2Cx_Config.I2C_DeviceAddress_Primary << 1));
	 }

	 //Dual Address Mode
	 if(pI2CHandler->I2Cx_Config.I2C_DeviceAddrMode == I2C_DeviceAddr_Dual)
	 {
		 pI2CHandler->pI2Cx->OAR2 |= ((pI2CHandler->I2Cx_Config.I2C_DeviceAddress_Secondary << 1) + 0x1 );
	 }

	 //CCR
	 if(pI2CHandler->I2Cx_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STD)			//Standard Mode
	 {
		 pI2CHandler->pI2Cx->CCR |= ((RCC_GetPeriCLK1_Value() / (2 * pI2CHandler->I2Cx_Config.I2C_SCLSpeed)) & 0xFFF);
	 }
	 else																	//Fast Mode
	 {
		 if(pI2CHandler->I2Cx_Config.I2C_FMDutyCycle == I2C_DUTY_2)
		 {
			 pI2CHandler->pI2Cx->CCR |= ((0x1 << 15) + (RCC_GetPeriCLK1_Value() / (3 * pI2CHandler->I2Cx_Config.I2C_SCLSpeed)));
		 }
		 else
		 {
			 pI2CHandler->pI2Cx->CCR |= ((0x3 << 14) + (RCC_GetPeriCLK1_Value() / (25 * pI2CHandler->I2Cx_Config.I2C_SCLSpeed)));
		 }

	 }

	 //Rise Time: TRISE = (Max_TRise / TPclk)+ 1 = (PCLK_Freq / TRise in MHz) + 1 = (PCLK_Freq * TRise) + 1
	 if(pI2CHandler->I2Cx_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STD)			//Standard Mode
	 {
		 pI2CHandler->pI2Cx->TRISE |= (((RCC_GetPeriCLK1_Value() / 1000000U) + 1) & 0x3F);
	 }
	 else
	 {
		 pI2CHandler->pI2Cx->TRISE |= ((((RCC_GetPeriCLK1_Value() * 300) / 1000000000U) + 1) & 0x3F);
	 }

}





/***********************************************************************************
 * 					 	I2C Deinitialize Handler
 *
 * @fn: 		- 	I2Cx_DeInit
 *
 * @brief		-	This function resets all the registers associated with that I2C Peripheral,
 * 					thus Deinitialize the given I2C Peripheral.
 *
 * @param[1]	-	Base Address of the I2C Peripheral
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void I2Cx_DeInit(I2Cx_RegDef_ty *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_PERI_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_PERI_RESET();
	}
	else
	{
		I2C3_PERI_RESET();
	}

}






/***********************************************************************************
 * 					 		I2C Flag Status Handler
 *
 * @fn: 		- 	I2Cx_GetFlagStatus
 *
 * @brief		-	Function which returns status of a flag of Status Register (SR).
 *
 * @param[1]	-	Base Address of the I2C Peripheral
 * @param[2]	-	Flag value from @I2C_FLAG_BITS
 *
 * @return		-	uint8_t
 *
 * @Note		-	TODO: SR2 (Status Register 2) Support.
 *
 */
uint8_t I2Cx_GetFlagStatus(I2Cx_RegDef_ty *pI2Cx, uint8_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return	SET;
	}
	else
	{
		return	RESET;
	}
}





/***********************************************************************************
 * 					 		I2C ACK Control Handler
 *
 * @fn: 		- 	I2Cx_Acking
 *
 * @brief		-	Function to Enable or Disable Auto Acknowledge Bit generation.
 *
 * @param[1]	-	Base Address of the I2C Peripheral
 * @param[2]	-	Control: ENABLE or DISABLE
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
void I2Cx_ACKControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control)
{
	if(Control == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 <<I2C_CR1_ACK);
	}
}






/***********************************************************************************
 * 					 		I2C Clear ADDR Flag Handler
 *
 * @fn: 		- 	I2Cx_ClearADDRFlag
 *
 * @brief		-	Function clears the ADDR Flag of SR1 Register.
 *
 * @param[1]	-	Base Address of the I2C Handler.
 *
 * @return		-	void
 *
 * @Note		-
 *
 */
static void I2Cx_ClearADDRFlag(I2Cx_Handler_ty *pI2CHandler)
{
	__UNUSED uint32_t dummyRead;

	//check if master mode
	if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//Rx
		if(pI2CHandler->TxRxState == I2C_BUSY_RX)
		{
			//single byte reception
			if(pI2CHandler->RxSize == 1)
			{
				//disable auto ACK
				I2Cx_ACKControl(pI2CHandler->pI2Cx, DISABLE);

				//clear ADDR by reading SR1 and SR2
				dummyRead = pI2CHandler->pI2Cx->SR1;
				dummyRead = pI2CHandler->pI2Cx->SR2;
			}

		}
		else
		{
			dummyRead = pI2CHandler->pI2Cx->SR1;
			dummyRead = pI2CHandler->pI2Cx->SR2;
		}
	}
	else
	{
		dummyRead = pI2CHandler->pI2Cx->SR1;
		dummyRead = pI2CHandler->pI2Cx->SR2;
	}




}






/***********************************************************************************
 * 					 	I2C Master Send Data Handler
 *
 * @fn: 		- 	I2Cx_SendData_Master
 *
 * @brief		-	This function handles data transmission of the given I2C
 * 					peripheral.
 *
 * @param[1]	-	Pointer to the I2C Peripheral Handler
 *
 * @param[2]	-	Transmission buffer
 *
 * @param[3]	-	Transmission data length
 *
 * @param[4]	-	Slave Address to which data is to be sent.
 *
 * @param[5]	-	Repeated START Configuration, I2C_SR_ENABLE/I2C_SR_DISABLE
 *
 * @return		-	void
 *
 * @Note		-	The Slave Address must be either 7-bit or 10-bit.
 * 					TODO: 10-bit slave address support
 * 					TODO: Continuous data write to slave
 *
 */
void I2Cx_SendData_Master(I2Cx_Handler_ty *pI2CHandler, uint8_t *pTxBuffer, uint8_t length, uint8_t SlaveAddr, uint8_t SR)
{
	//Generate START condition
	{
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	}

	//Confirm START generation is completed by checking SB flag in SR1
	{
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));
	}

	//Send Address of target slave + R/W = 0
	{
		pI2CHandler->pI2Cx->DR = ((SlaveAddr << 1) & (~(1)));			//free LSB of address byte and put 0 for R/W = 0
	}

	//Confirm address is sent by checking ADDR flag in SR1
	{
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));
	}

	//clear ADDR Flag
	I2Cx_ClearADDRFlag(pI2CHandler);

	//Send Data
	{
		while(length > 0)
		{
			while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxE));		//wait till TxE is set
			pI2CHandler->pI2Cx->DR = *pTxBuffer;
			pTxBuffer++;
			length--;
		}
	}

	//wait for TxE=1 and BTF=1 ie SR and DR empty
	{
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxE));
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BTF));
	}

	//Generate STOP Condition
	{
		if(SR == I2C_SR_DISABLE)
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}







/***********************************************************************************
 * 					 	I2C Master Receive Data Handler
 *
 * @fn: 		- 	I2Cx_ReceiveData_Master
 *
 * @brief		-	This function handles data reception by the master of the given I2C
 * 					peripheral.
 *
 * @param[1]	-	Pointer to the I2C Peripheral Handler
 *
 * @param[2]	-	Reception buffer
 *
 * @param[3]	-	Receiving data length
 *
 * @param[4]	-	Slave Address from which data is to be received.
 *
 * @param[5]	-	Repeated START Configuration, I2C_SR_ENABLE/I2C_SR_DISABLE
 *
 * @return		-	void
 *
 * @Note		-	The Slave Address must be either 7-bit or 10-bit.
 * 					TODO: 10-bit slave address support
 * 					TODO: Continuous data read from slave
 *
 */
void I2Cx_ReceiveData_Master(I2Cx_Handler_ty *pI2CHandler, uint8_t *pRxBuffer, uint8_t length, uint8_t SlaveAddr, uint8_t SR)
{
	//Generate START
	pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	//Verify START generation
	while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));

	//Send Address + R/W = 1
	pI2CHandler->pI2Cx->DR = ((SlaveAddr << 1) | 1);

	//Confirm Address generation
	while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));

	//Read One Byte if Length=1
	if(length == 1)
	{
		//Disable ACK
		I2Cx_ACKControl(pI2CHandler->pI2Cx, DISABLE);

		//Clear ADDR flag
		I2Cx_ClearADDRFlag(pI2CHandler);

		//Wait till RXNE=1
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RxNE));

		//Generate STOP
		if(SR == I2C_SR_DISABLE)
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//Read Data from DR
		*pRxBuffer = pI2CHandler->pI2Cx->DR;

	}

	//Read More then One Byte if Length>1
	else
	{
		//Clear ADDR flag
		I2Cx_ClearADDRFlag(pI2CHandler);

		for(uint32_t var=length; var>0; var--)
		{
			//Wait till RXNE=1
			while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RxNE));

			//if last 2 bytes remaining
			if(var == 2)
			{
				//Disable ACK
				I2Cx_ACKControl(pI2CHandler->pI2Cx, DISABLE);

				//Generate STOP
				if(SR == I2C_SR_DISABLE)
				pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			}

			//Read from DR
			*pRxBuffer = pI2CHandler->pI2Cx->DR;

			//Increment buffer address
			pRxBuffer++;

		}
	}

	//Restore ACK Config
	if(pI2CHandler->I2Cx_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2Cx_ACKControl(pI2CHandler->pI2Cx, ENABLE);
	}

}






/***********************************************************************************
 * 					Interrupt based I2C Master Transmission Data Handler
 *
 * @fn: 		- 	I2Cx_SendData_MasterIT
 *
 * @brief		-	This function handles data transmission by the master of the given I2C
 * 					peripheral based on interrupt.
 *
 * @param[1]	-	Pointer to the I2C Peripheral Handler
 *
 * @param[2]	-	Transmission buffer
 *
 * @param[3]	-	Transmitting  data length
 *
 * @param[4]	-	Slave Address to which data is to be sent
 *
 * @param[5]	-	Repeated START Configuration, I2C_SR_ENABLE/I2C_SR_DISABLE
 *
 * @return		-	uint8_t
 *
 * @Note		-	The Slave Address must be either 7-bit or 10-bit.
 * 					TODO: 10-bit slave address support
 * 					TODO: Continuous data read from slave
 *
 */
uint8_t I2Cx_SendData_MasterIT(I2Cx_Handler_ty *pI2CHandler, uint8_t *pTxBuffer, uint8_t length, uint8_t SlaveAddr, uint8_t SR)
{
	if((pI2CHandler->TxRxState != I2C_BUSY_TX) && (pI2CHandler->TxRxState != I2C_BUSY_RX))
	{
		pI2CHandler->pTxBuffer = pTxBuffer;
		pI2CHandler->TxLength = length;
		pI2CHandler->TxRxState = I2C_BUSY_TX;
		pI2CHandler->Device_Slave_ADDR = SlaveAddr;
		pI2CHandler->SR = SR;

		//Generate START
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Enable ITBUFEN (Buffer Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVFEN (Event Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN (Error Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return (pI2CHandler->TxRxState);
}




/***********************************************************************************
 * 					Interrupt based I2C Master Receive Data Handler
 *
 * @fn: 		- 	I2Cx_ReceiveData_MasterIT
 *
 * @brief		-	This function handles data reception by the master of the given I2C
 * 					peripheral based on interrupt.
 *
 * @param[1]	-	Pointer to the I2C Peripheral Handler
 *
 * @param[2]	-	Reception buffer
 *
 * @param[3]	-	Receiving data length
 *
 * @param[4]	-	Slave Address from which data is to be received
 *
 * @param[5]	-	Repeated START Configuration, I2C_SR_ENABLE/I2C_SR_DISABLE
 *
 * @return		-	uint8_t
 *
 * @Note		-	The Slave Address must be either 7-bit or 10-bit.
 * 					TODO: 10-bit slave address support
 * 					TODO: Continuous data read from slave
 *
 */
uint8_t I2Cx_ReceiveData_MasterIT(I2Cx_Handler_ty *pI2CHandler, uint8_t *pRxBuffer, uint8_t length, uint8_t SlaveAddr, uint8_t SR)
{
	if((pI2CHandler->TxRxState != I2C_BUSY_TX) && (pI2CHandler->TxRxState != I2C_BUSY_RX))
	{
		pI2CHandler->pRxBuffer = pRxBuffer;
		pI2CHandler->RxLength = length;
		pI2CHandler->TxRxState = I2C_BUSY_RX;
		pI2CHandler->Device_Slave_ADDR = SlaveAddr;
		pI2CHandler->SR = SR;

		//Generate START
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Enable ITBUFEN (Buffer Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVFEN (Event Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN (Error Interrupt Enable) Bit in CR2
		pI2CHandler->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return (pI2CHandler->TxRxState);
}




/***********************************************************************************
 * 					 	I2C Close Interrupt based Rx Handler
 *
 * @fn: 		- 	I2Cx_Close_ReceiveData
 *
 * @brief		-	This function closes the interrupt based I2C Reception.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 *
 */
void I2Cx_Close_INTRReceiveData_(I2Cx_Handler_ty *pI2CHandler)
{
	//Disable ITBUFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandler->TxRxState = I2C_READY;
	pI2CHandler->pRxBuffer = NULL;
	pI2CHandler->RxLength = 0;
	pI2CHandler->RxSize = 0;
	pI2CHandler->SR = RESET;

	//Restore ACK Config
	if(pI2CHandler->I2Cx_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	I2Cx_ACKControl(pI2CHandler->pI2Cx, ENABLE);
}





/***********************************************************************************
 * 					 	I2C Close Interrupt based Tx Handler
 *
 * @fn: 		- 	I2Cx_Close_INTRSendData
 *
 * @brief		-	This function closes the Interrupt based I2C Transmission.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 *
 */
void I2Cx_Close_INTRSendData(I2Cx_Handler_ty *pI2CHandler)
{
	//Disable ITBUFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVFEN
	pI2CHandler->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandler->TxRxState = I2C_READY;
	pI2CHandler->TxLength = 0;
	pI2CHandler->pTxBuffer = NULL;
}






/***********************************************************************************
 * 					 	I2C Master TxE Handler
 *
 * @fn: 		- 	I2Cx_MasterHandle_TXEInterrupt
 *
 * @brief		-	This function handles the interrupts caused by setting of
 * 					TxE flag.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 *
 */
static void I2Cx_MasterHandle_TXEInterrupt(I2Cx_Handler_ty *pI2CHandler)
{
	if(pI2CHandler->TxLength > 0)
	{
		//load DR
		pI2CHandler->pI2Cx->DR = *(pI2CHandler->pTxBuffer);

		//decrement Tx length
		(pI2CHandler->TxLength)--;

		//Increment Tx pointer
		(pI2CHandler->pTxBuffer)++;
	}
}





/***********************************************************************************
 * 					 	I2C Master RxNE Handler
 *
 * @fn: 		- 	I2Cx_MasterHandle_RXNEInterrupt
 *
 * @brief		-	This function handles the interrupts caused by setting of
 * 					RxNE flag.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-
 *
 *
 */
static void I2Cx_MasterHandle_RXNEInterrupt(I2Cx_Handler_ty *pI2CHandler)
{
	if(pI2CHandler->RxLength == 1)
	{
		pI2CHandler->pI2Cx->DR = *(pI2CHandler->pRxBuffer);
		(pI2CHandler->RxLength)--;
	}

	else if(pI2CHandler->RxLength > 1)
	{
		if(pI2CHandler->RxLength == 2)
		{
			I2Cx_ACKControl(pI2CHandler->pI2Cx, DISABLE);
		}

		//Read from DR
		pI2CHandler->pI2Cx->DR = *(pI2CHandler->pRxBuffer);
		pI2CHandler->pRxBuffer++;
		pI2CHandler->RxLength--;

	}

	else
	{
		if(pI2CHandler->SR == I2C_SR_DISABLE)
		//generate STOP
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//Close Rx
		I2Cx_Close_INTRReceiveData_(pI2CHandler);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_EV_RX_CMPLT);
	}
}






/***********************************************************************************
 * 					 	I2C Event Interrupt Handler
 *
 * @fn: 		- 	I2Cx_EV_IRQHandling
 *
 * @brief		-	This function handles the interrupts caused by various events
 * 					given below.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-	This interrupt is generated when:
 *									SB = 1 (Master)
 *									ADDR = 1 (Master/Slave)
 *								 	ADD10= 1 (Master)
 *									STOPF = 1 (Slave)
 *									BTF = 1 with no TxE or RxNE event
 *									TxE = 1 if ITBUFEN = 1
 *									RxNE = 1 if ITBUFEN = 1
 *
 *					TODO: 10bit Address support
 *
 *
 */
void I2Cx_EV_IRQHandling(I2Cx_Handler_ty *pI2CHandler)
{
	//Event due to SB=1, applicable only in master mode
	if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB)))
	{
		if(pI2CHandler->TxRxState == I2C_BUSY_TX)
		{
			pI2CHandler->pI2Cx->DR = ((pI2CHandler->Device_Slave_ADDR << 1) & (~(1)));			//free LSB of address byte and put 0 for R/W = 0
		}
		else if(pI2CHandler->TxRxState == I2C_BUSY_RX)
		{
			pI2CHandler->pI2Cx->DR = ((pI2CHandler->Device_Slave_ADDR << 1) | 1);				//free LSB of address byte and put 0 for R/W = 1
		}
	}

	/* Event due to ADDR=1, applicable for both master/slave mode
	 * This event is triggered when:
	 * 	   	   	   	   	   	  1. Address is sent and ACK is received (7-bit address/Master mode)
	 * 	   	   	   	   	   	  2. 2nd Byte is sent of address is sent and ACK is received (10-bit address/Master mode)
	 * 	   	   	   	   	   	  3. Address is matched with own device address (Slave mode)
	 */
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR)))
	{
		I2Cx_ClearADDRFlag(pI2CHandler);
	}

	//Event due to ADD10 = 1, applicable only in master mode
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADD10)))
	{
		//ADD10 is set TODO
	}

	//Event due to BTF = 1
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BTF)))
	{
		if(pI2CHandler->TxRxState == I2C_BUSY_TX)
		{
			//Check TxE=1
			if(I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxE))
			{
				//check length of transmission is 0
				if(pI2CHandler->TxLength == 0)
				{
					//check if repeated START is disabled
					if(pI2CHandler->SR == I2C_SR_DISABLE)
					//generate STOP
					pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

					//resert all handler members
					I2Cx_Close_INTRSendData(pI2CHandler);

					//notify the application that Tx is complete
					I2Cx_ApplicationEventCallback(pI2CHandler, I2C_EV_TX_CMPLT);
				}
			}
		}
		/*else if(pI2CHandler->TxRxState == I2C_BUSY_RX)
		{
			if I2C is busy in Rx then its not in out hand to generate STOP

		}*/
	}

	//Event due to STOPF = 1 (STOP is detected), applicable only in slave mode
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_STOPF)))
	{
		//clear STOPF, Read SR1 (already done above) then write to CR1
		pI2CHandler->pI2Cx->CR1 |= 0x0000;

		//Notify application that STOP is detected
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_EV_STOP);
	}

	//Event due to TxE = 1, applicable for both master/slave mode
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN)
															   && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxE)))
	{
		//check whether I2Cx is in master mode
		if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//DR is Empty
			if(pI2CHandler->TxRxState == I2C_BUSY_TX)
			{
				I2Cx_MasterHandle_TXEInterrupt(pI2CHandler);
			}
		}
	}

	//Event due to RxNE = 1, applicable for both master/slave mode
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN)) && pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN)
															   && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RxNE)))
	{
		//check I2Cx is in master mode
		if(pI2CHandler->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//check RxNE is set
			if(I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_RxNE))
			{
				I2Cx_MasterHandle_RXNEInterrupt(pI2CHandler);
			}
		}
	}
}






/***********************************************************************************
 * 					 	I2C Error Interrupt Handler
 *
 * @fn: 		- 	I2Cx_ER_IRQHandling
 *
 * @brief		-	This function handles the interrupts caused by various errors
 * 					given below.
 *
 * @param[1]	-	Pointer I2C handler
 *
 * @return		-	void
 *
 * @Note		-	This interrupt is generated when:
 *										 	BERR = 1
 *										 	ARLO = 1
 *										 	AF = 1
 *										 	OVR = 1
 *										 	PECERR = 1
 *										 	TIMEOUT = 1
 *										 	SMBALERT = 1
 *
 */
void I2Cx_ER_IRQHandling(I2Cx_Handler_ty *pI2CHandler)
{
	//Bus Error
	if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BERR)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_BEER);
	}

	//Arbitration loss (Master)
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ARLO)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_ARLO);
	}

	//Acknowledge failure
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_AF)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_AF);
	}

	//Overrun/Underrun
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_OVR)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_OVR);
	}

	//PEC error
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_PECERR)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_PECERR);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_PECERR);
	}

	//Timeout/Tlow error
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TIMEOUT)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_TIMEOUT);
	}

	//SMBus Alert
	else if((pI2CHandler->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN)) && (I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SMBALERT)))
	{
		//clear BEER flag
		pI2CHandler->pI2Cx->SR1 &= ~(1 << I2C_SR1_SMBALERT);

		//Notify the application
		I2Cx_ApplicationEventCallback(pI2CHandler, I2C_ER_SMBALERT);
	}
}






/***********************************************************************************
 * 					 	I2C Application Even Callback Handler
 *
 * @fn: 		- 	I2Cx_ApplicationEventCallback
 *
 * @brief		-	This function handles the callback after an application event
 * 					is occurred.
 *
 * @param[1]	-	Pointer to I2C Handler
 *
 * @param[2]	-	Application Event
 *
 * @return		-	void
 *
 * @Note		-	This is a weak implementation, and should be implemented in user
 * 					application
 *
 */
__WEAK void I2Cx_ApplicationEventCallback(I2Cx_Handler_ty *pI2CHandler, uint8_t AppEvent)
{
	//weak implementation, application needs to implement it
}


