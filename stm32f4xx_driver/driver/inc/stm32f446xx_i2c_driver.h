
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
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 16-June-2020
 *      Author: Rajssss@GitHub.com
 *      Description: I2C Driver for STM32F446RE-Nucleo Board
 *
 *      This file contains:
 *           - Data structures and the address mapping for all I2C Peripherals
 *           - I2C configuration registers declarations and bits definition
 *           - Macros to access various I2C configuration Registers
 *           - Macros to various I2C configuration Registers Mode and options
 *           - Function prototypes to perform various operation on a I2C Bus
 *
 */


#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include  "stm32f446xx.h"


/*****************************************************************************************
 * I2Cx Peripheral Configuration structure.
 *****************************************************************************************/
typedef struct
{
	uint32_t 		I2C_SCLSpeed;					//@I2C_SCLSpeed
	uint8_t			I2C_DeviceAddrMode;				//@Device_Addr_Mode (10/7-bit, Only for Primary address)
	uint8_t			I2C_DualDeviceAddrConfig;		//Dual Device address config @Dual_Device_Addr
	uint8_t 		I2C_DeviceAddress_Primary;		//Primary Device Own Address
	uint8_t 		I2C_DeviceAddress_Secondary;	//Secondary Device Own Address
	uint8_t 		I2C_ACKControl;					//@ACKControl
	uint16_t		I2C_FMDutyCycle;				//@DutyCycle

}I2Cx_Config_ty;



/*****************************************************************************************
 * I2Cx Handler structure.
 *****************************************************************************************/
typedef struct
{
	I2Cx_RegDef_ty *pI2Cx;
	I2Cx_Config_ty I2Cx_Config;

}I2Cx_Handler_ty;




/*****************************************************************************************
 * I2Cx Maximum Speeds for different Modes. @I2C_SCLSpeed
 * Note: These speeds are not fixed, you can take any values between them.
 *****************************************************************************************/
#define 	I2C_SCL_SPEED_STD		100000
#define 	I2C_SCL_SPEED_FAST		400000




/*****************************************************************************************
 * I2Cx Auto Acknowledgment configuration macros. @ACKControl
 *****************************************************************************************/
#define		I2C_ACK_ENABLE			1
#define 	I2C_ACK_DISABLE			0




/*****************************************************************************************
 * I2Cx Available Duty Cycles. @DutyCycle
 *****************************************************************************************/
#define 	I2C_DUTY_2				0
#define 	I2C_DUTY_16_9			1



/*****************************************************************************************
 * I2Cx Device Address Mode. @Device_Addr_Mode
 *****************************************************************************************/
#define 	I2C_DeviceAddr_7BIT			0
#define 	I2C_DeviceAddr_10BIT		1




/*****************************************************************************************
 * I2Cx  Dual Device Address Configs. @Dual_Device_Addr
 *****************************************************************************************/
#define 	I2C_DeviceAddr_Single		0
#define 	I2C_DeviceAddr_Dual			1





/*****************************************************************************************
 * I2Cx Status Flags macros for bit-masking. @I2C_FLAG_BITS
 *****************************************************************************************/
#define 	I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define 	I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define 	I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define 	I2C_FLAG_ADD10				(1 << I2C_SR1_ADD10)
#define 	I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define 	I2C_FLAG_RxNE				(1 << I2C_SR1_RXNE)
#define 	I2C_FLAG_TxE				(1 << I2C_SR1_TXE)
#define 	I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define 	I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define 	I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define 	I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define 	I2C_FLAG_PECERR				(1 << I2C_SR1_PECERR)
#define 	I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)
#define 	I2C_FLAG_SMBALERT			(1 << I2C_SR1_SMBALERT)

#define 	I2C_FLAG_MSL				(1 << I2C_SR2_MSL)
#define 	I2C_FLAG_BUSY				(1 << I2C_SR2_BUSY)
#define 	I2C_FLAG_TRA				(1 << I2C_SR2_TRA)
#define 	I2C_FLAG_GENCALL			(1 << I2C_SR2_GENCALL)
#define 	I2C_FLAG_SMB_DEFAULT		(1 << I2C_SR2_SMBDEFAULT)
#define 	I2C_FLAG_SMB_HOST			(1 << I2C_SR2_SMBHOST)
#define 	I2C_FLAG_DUALF				(1 << I2C_SR2_DUALF)





/*****************************************************************************************
 * 							APIs Supported by this Driver
 * 				For more information about the APIs check definition.
*****************************************************************************************/
//I2C Clock Control Handler
void I2Cx_PeriClkControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control);


//I2C Initialize/Deinitialize Handler
void I2Cx_Init(I2Cx_Handler_ty *pI2CHandler);
void I2Cx_DeInit(I2Cx_RegDef_ty *pI2Cx);


//I2C Data Send and Receive handlers
void I2Cx_SendData_Master(I2Cx_Handler_ty *pI2CHandler, uint8_t *pTxBuffer, uint8_t length, uint8_t SlaveAddr);
void I2Cx_ReceiveData_Master(I2Cx_Handler_ty *pI2CHandler, uint8_t *pTxBuffer, uint8_t length, uint8_t SlaveAddr);



//Other handlers
uint8_t I2Cx_GetFlagStatus(I2Cx_RegDef_ty *pI2Cx, uint8_t FlagName);
void I2Cx_PeripheralControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control);
void I2Cx_ACKControl(I2Cx_RegDef_ty *pI2Cx, uint8_t Control);



#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
