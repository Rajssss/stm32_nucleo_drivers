
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
 * @param[1]	-	Base Address of the SPI Peripheral
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
 * @return		-	void
 *
 * @Note		-	The Slave Address must be either 7-bit or 10-bit.
 * 					TODO: 10-bit slave address support
 *
 */
void I2Cx_SendData_Master(I2Cx_Handler_ty *pI2CHandler, uint8_t *pTxBuffer, uint8_t length, uint8_t SlaveAddr)
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
		pI2CHandler->pI2Cx->DR = (((SlaveAddr << 1) & (~(1))) );			//free LSB of address byte and put 0 for R/W = 0
	}

	//Confirm address is sent by checking ADDR flag in SR1
	{
		while(! I2Cx_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));
	}

	//Clear ADDR flag before
	{
		__UNUSED uint32_t dummyRead;				//Dummy read SR1 and SR2 to clear ADDR Flag

		dummyRead = pI2CHandler->pI2Cx->SR1;
		dummyRead = pI2CHandler->pI2Cx->SR2;
	}

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
		pI2CHandler->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}




