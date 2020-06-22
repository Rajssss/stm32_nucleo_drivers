
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



/***********************************************************************************
 * 					 	I2C Clock Control Handler
 *
 * @fn: 		- 	I2Cx_PeriClkControl
 *
 * @brief		-	This function Deinitialize the given I2C Port
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
 * @Note		-	TODO: 1. Dual Own Address Mode
 *
 */
void I2Cx_Init(I2Cx_Handler_ty *pI2CHandler)
{
	//ACK Control
	pI2CHandler->pI2Cx->CR1 |= (pI2CHandler->I2Cx_Config.I2C_ACKControl << I2C_CR1_ACK);

	//Frequency
	 pI2CHandler->pI2Cx->CR2 |= (RCC_GetPeriCLK1_Value() / 1000000U & 0x3F);


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

}


