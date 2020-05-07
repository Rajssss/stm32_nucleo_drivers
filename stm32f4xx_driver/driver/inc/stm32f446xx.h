
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
 * stm32f446xx.h
 *
 *  Created on: May 4, 2020
 *      Author: Rajssss@GitHub.com
 *
 *      Description: Device Peripheral Access Layer Header for STM32F446xx.
 *
 *      This file contains:
 *           - Data structures and the address mapping for all peripherals
 *           - peripherals registers declarations and bits definition
 *           - Macros to access peripheral’s registers hardware
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>


//Generic Macros
#define 	__VOL 	volatile

#define 	SET			 1
#define 	RESET	 	 0
#define 	HIGH	 	 SET
#define 	LOW		 	 RESET
#define 	ENABLE		 SET
#define		DISABLE		 RESET


/**********************************************************************************
 * Base Address of different memory chips
***********************************************************************************/
#define 	FLASH_BASEADDR					0x08000000U				//Flash Size
#define 	SRAM1_BASEADDR					0x20000000U    			//SRAM1 Size
#define 	SRAM_BASEADDR					SRAM1_BASEADDR			//SRAM1 is primary RAM
#define 	SRAM2_BASSADDR					0x2001C000U				//SRAM2 Size
#define		ROM								0x1FFF0000U				//System memory size


/**********************************************************************************
 * AHBx and APBx Peripherals Base Addresses
***********************************************************************************/
#define 	PERIPH_BASE						0x40000000U
#define 	APB1PERIPH_BASEADDR				PERIPH_BASE
#define 	APB2PERIPH_BASEADDR				0x40010000U
#define 	AHB1PERIPH_BASEADDR				0x40020000U
#define 	AHB2PERIPH_BASEADDR				0x50000000U



/**********************************************************************************
 * AHB1 Peripherals Memory Map
***********************************************************************************/
#define 	GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0000U)
#define 	GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0400U)
#define 	GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0800U)
#define 	GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0C00U)
#define 	GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1000U)
#define 	GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1400U)
#define 	GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1800U)
#define 	GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1C00U)
#define 	CRC_BASEADDR					(AHB1PERIPH_BASEADDR + 0X3000U)
#define 	RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0X3800U)
#define 	FLASH_REGS_BASEADDR				(AHB1PERIPH_BASEADDR + 0X3C00U)
#define 	BKPSRAM_BASEADDR				(AHB1PERIPH_BASEADDR + 0X4000U)
#define 	DMA1_BASEADDR					(AHB1PERIPH_BASEADDR + 0X6000U)
#define 	DMA2_BASEADDR					(AHB1PERIPH_BASEADDR + 0X6400U)
#define 	USBOTG_HS_BASEADDR				(AHB1PERIPH_BASEADDR + 0x20000U)



/**********************************************************************************
 * APB1 Peripherals Memory Map
***********************************************************************************/
#define 	TIM2_BASEADDR					(APB1PERIPH_BASEADDR + 0X0000U)
#define 	TIM3_BASEADDR					(APB1PERIPH_BASEADDR + 0X0400U)
#define 	TIM4_BASEADDR					(APB1PERIPH_BASEADDR + 0X0800U)
#define 	TIM5_BASEADDR					(APB1PERIPH_BASEADDR + 0X0C00U)
#define 	TIM6_BASEADDR					(APB1PERIPH_BASEADDR + 0X1000U)
#define 	TIM7_BASEADDR					(APB1PERIPH_BASEADDR + 0X1400U)
#define 	TIM12_BASEADDR					(APB1PERIPH_BASEADDR + 0X1800U)
#define 	TIM13_BASEADDR					(APB1PERIPH_BASEADDR + 0X1C00U)
#define 	TIM414_BASEADDR					(APB1PERIPH_BASEADDR + 0X2000U)
#define 	RTC_BKP_REGS_BASEADDR			(APB1PERIPH_BASEADDR + 0X2800U)
#define 	WWDG_BASEADDR					(APB1PERIPH_BASEADDR + 0X2C00U)
#define 	IWDG_BASEADDR					(APB1PERIPH_BASEADDR + 0X3000U)
#define 	SPI2_I2S2_BASEADDR				(APB1PERIPH_BASEADDR + 0X3800U)
#define 	SPI3_I2S3_BASEADDR				(APB1PERIPH_BASEADDR + 0X3C00U)
#define 	SPDIF_RX_BASEADDR				(APB1PERIPH_BASEADDR + 0x4000U)
#define 	USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0X4400U)
#define 	USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0X4800U)
#define 	UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0X4C00U)
#define 	UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0X5000U)
#define 	I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0X5400U)
#define 	I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0X5C00U)
#define 	CAN1_BASEADDR					(APB1PERIPH_BASEADDR + 0X6400U)
#define 	CAN2_BASEADDR					(APB1PERIPH_BASEADDR + 0X6800U)
#define 	HDMI_CEC_BASEADDR				(APB1PERIPH_BASEADDR + 0X6C00U)
#define 	PWR_BASEADDR					(APB1PERIPH_BASEADDR + 0X7000U)
#define 	DAC_BASEADDR					(APB1PERIPH_BASEADDR + 0X7400U)




/**********************************************************************************
 * APB2 Peripherals Memory Map
***********************************************************************************/
#define 	TIM1_BASEADDR					(APB2PERIPH_BASEADDR + 0X0000U)
#define 	TIM8_BASEADDR					(APB2PERIPH_BASEADDR + 0X0400U)
#define 	USART1_BASEADDR					(APBP2ERIPH_BASEADDR + 0X1000U)
#define 	USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0X1400U)
#define 	ADC_1_3_BASEADDR				(APB2PERIPH_BASEADDR + 0X2000U)
#define 	SDMMC_BASEADDR					(APB2PERIPH_BASEADDR + 0X2C00U)
#define 	SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0X3000U)
#define 	SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0X3400U)
#define 	SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0X3800U)
#define 	EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0X3C00U)
#define 	TIM9_BASEADDR					(APB2PERIPH_BASEADDR + 0X4000U)
#define 	TIM10_BASEADDR					(APB2PERIPH_BASEADDR + 0X4400U)
#define 	TIM11_BASEADDR					(APB2PERIPH_BASEADDR + 0X4800U)
#define 	SAI1_BASEADDR					(APB2PERIPH_BASEADDR + 0X5800U)
#define 	SAI2_RX_BASEADDR				(APB2PERIPH_BASEADDR + 0x5C00U)





/**********************************************************************************
 * GPIOx Peripheral Registers Structure
***********************************************************************************/
typedef struct
{
	__VOL uint32_t MODER;					//GPIO port mode register
	__VOL uint32_t OTYPER;					//GPIO port output type register
	__VOL uint32_t OSPEEDER;				//GPIO port output speed register
	__VOL uint32_t PUPDR;					//GPIO port pull-up/pull-down register
	__VOL uint32_t IDR;						//GPIO port input data register
	__VOL uint32_t ODR;						//GPIO port output data register
	__VOL uint32_t BSRR;					//GPIO port bit set/reset register
	__VOL uint32_t LCKR;					//GPIO port configuration lock register
	__VOL uint32_t AFR[2];					//AFR[0]: GPIO alternate function low register
											//AFR[1]: GPIO alternate function high register

}GPIOx_RegDef_ty;




/**********************************************************************************
 * RCC Peripheral Registers Structure
***********************************************************************************/
typedef struct
{
	__VOL uint32_t CR;						//RCC clock control register
	__VOL uint32_t PLL;						//RCC PLL configuration register
	__VOL uint32_t CFGR;					//RCC clock configuration register
	__VOL uint32_t CIR;						//RCC clock interrupt register
	__VOL uint32_t AHB1RSTR;				//RCC AHB1 peripheral reset register
	__VOL uint32_t AHB2RSTR;				//RCC AHB2 peripheral reset register
	__VOL uint32_t AHB3RSTR;				//RCC AHB3 peripheral reset register
		  uint32_t RESERVED0;				//Reserved Memory Region
	__VOL uint32_t APB1RSTR;				//RCC APB1 peripheral reset register
	__VOL uint32_t APB2RSTR;				//RCC APB2 peripheral reset register
	      uint32_t RESERVED1[2];			//Reserved Memory Region
	__VOL uint32_t AHB1ENR;					//RCC AHB1 peripheral clock enable register
	__VOL uint32_t AHB2ENR;					//RCC AHB2 peripheral clock enable register
	__VOL uint32_t AHB3ENR;					//RCC AHB3 peripheral clock enable register
	      uint32_t RESERVED2;				//Reserved Memory Region
	__VOL uint32_t APB1ENR;					//RCC APB1 peripheral clock enable register
	__VOL uint32_t APB2ENR;					//RCC APB2 peripheral clock enable register
	      uint32_t RESERVED3[2];			//Reserved Memory Region
	__VOL uint32_t AHB1LPENR;				//RCC AHB1 peripheral clock enable in low power mode register
	__VOL uint32_t AHB2LPENR;				//RCC AHB2 peripheral clock enable in low power mode register
	__VOL uint32_t AHB3LPENR;				//RCC AHB3 peripheral clock enable in low power mode register
	      uint32_t RESERVED4;				//Reserved Memory Region
	__VOL uint32_t APB1LPENR;				//RCC APB1 peripheral clock enable in low power mode register
	__VOL uint32_t APB2LPENR;				//RCC APB2 peripheral clock enable in low power mode register
	      uint32_t RESERVED5[2];			//Reserved Memory Region
	__VOL uint32_t BDCR;					//RCC Backup domain control register
	__VOL uint32_t CSR;						//RCC clock control & status register
	      uint32_t RESERVED6[2];			//Reserved Memory Region
	__VOL uint32_t SSCGR;					//RCC spread spectrum clock generation register
	__VOL uint32_t PLLI2SCFGR;				//RCC PLLI2S configuration register
	__VOL uint32_t PLLSAICFGR;				//RCC Dedicated Clock Configuration Register
	__VOL uint32_t DCKCFGR;					//RCC clocks gated enable register
	__VOL uint32_t DCKGATENR;				//RCC clocks gated enable register
	__VOL uint32_t DCKCFGR2;				//RCC dedicated clocks configuration register 2


}RCC_RegDef_ty;




/**********************************************************************************
 * Peripheral Definition (Peripheral base address typecasted to xxxx_Reg_Def_ty).
***********************************************************************************/
#define		GPIOA				((GPIOx_RegDef_ty* )GPIOA_BASEADDR)			//0x4002 0400
#define		GPIOB				((GPIOx_RegDef_ty* )GPIOB_BASEADDR)			//0x4002 0800
#define		GPIOC				((GPIOx_RegDef_ty* )GPIOC_BASEADDR) 		//0x4002 0C00
#define		GPIOD				((GPIOx_RegDef_ty* )GPIOD_BASEADDR)			//0x4002 1000
#define		GPIOE				((GPIOx_RegDef_ty* )GPIOE_BASEADDR)			//0x4002 1400
#define		GPIOF				((GPIOx_RegDef_ty* )GPIOF_BASEADDR)			//0x4002 1800
#define		GPIOG				((GPIOx_RegDef_ty* )GPIOG_BASEADDR)			//0x4002 1C00
#define		GPIOH				((GPIOx_RegDef_ty* )GPIOH_BASEADDR)			//0x4002 3000

#define     RCC    				((RCC_RegDef_ty* )RCC_BASEADDR)





/**********************************************************************************
 * Clock Enable Macro Definitions for GPIOx Peripherals
***********************************************************************************/
#define 	GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define 	GPIOB_PCLK_EN()	 		(RCC->AHB1ENR |= (1<<1))
#define 	GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define 	GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define 	GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define 	GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define 	GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define 	GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))


/**********************************************************************************
 * Clock Disable Macro Definitions for GPIOx Peripherals
***********************************************************************************/
#define 	GPIOA_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<0))
#define 	GPIOB_PCLK_DE()	 		(RCC->AHB1ENR &= ~(1<<1))
#define 	GPIOC_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<2))
#define 	GPIOD_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<3))
#define 	GPIOE_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<4))
#define 	GPIOF_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<5))
#define 	GPIOG_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<6))
#define 	GPIOH_PCLK_DE()			(RCC->AHB1ENR &= ~(1<<7))




/**********************************************************************************
 * Clock Disable Macro Definitions for GPIOx Peripherals
***********************************************************************************/
#define 	GPIOA_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 0));	((RCC->AHB1RSTR)&= ~(1 << 0)); }while(0)
#define 	GPIOB_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 1));	((RCC->AHB1RSTR)&= ~(1 << 1)); }while(0)
#define 	GPIOC_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 2));	((RCC->AHB1RSTR)&= ~(1 << 2)); }while(0)
#define 	GPIOD_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 3));	((RCC->AHB1RSTR)&= ~(1 << 3)); }while(0)
#define 	GPIOE_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 4));	((RCC->AHB1RSTR)&= ~(1 << 4)); }while(0)
#define 	GPIOF_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 5));	((RCC->AHB1RSTR)&= ~(1 << 5)); }while(0)
#define 	GPIOG_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 6));	((RCC->AHB1RSTR)&= ~(1 << 6)); }while(0)
#define 	GPIOH_REG_RESET()		do{ ((RCC->AHB1RSTR)|= (1 << 7));	((RCC->AHB1RSTR)&= ~(1 << 7)); }while(0)




/**********************************************************************************
 * Clock Enable Macro Definitions for various Peripherals of APB1 Bus.
***********************************************************************************/
#define 	TIM2_PCLK_EN()			(RCC->APB1ENR |= (1<<0))
#define 	TIM3_PCLK_EN()			(RCC->APB1ENR |= (1<<1))
#define 	TIM4_PCLK_EN()			(RCC->APB1ENR |= (1<<2))
#define 	TIM5_PCLK_EN()			(RCC->APB1ENR |= (1<<3))
#define 	TIM6_PCLK_EN()			(RCC->APB1ENR |= (1<<4))
#define 	TIM7_PCLK_EN()			(RCC->APB1ENR |= (1<<5))
#define 	TIM12_PCLK_EN()			(RCC->APB1ENR |= (1<<6))
#define 	TIM13_PCLK_EN()			(RCC->APB1ENR |= (1<<7))
#define 	TIM14_PCLK_EN()			(CC->APB1ENR |= (1<<8))

#define 	WWDG_PCLK_EN()			(RCC->APB1ENR |= (1<<11))

#define 	SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define 	SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))

#define 	SPDIFRX_PCLK_EN()		(RCC->APB1ENR |= (1<<16))

#define 	USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define 	USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))

#define 	UART4_PCLK_EN()			(RCC->APB1 |= (1<<19))
#define 	UART5_PCLK_EN()			(RCC->APB1 |= (1<<20))
#define 	I2C1_PCLK_EN()			(RCC->APB1 |= (1<<21))
#define 	I2C2_PCLK_EN()			(RCC->APB1 |= (1<<22))
#define 	I2C3_PCLK_EN()			(RCC->APB1 |= (1<<23))

#define 	FMPI2C1_PCLK_EN()		(RCC->APB1 |= (1<<24))
#define 	CAN1_PCLK_EN()			(RCC->APB1 |= (1<<25))
#define 	CAN2_PCLK_EN()			(RCC->APB1 |= (1<<26))
#define 	CEC_PCLK_EN()			(RCC->APB1 |= (1<<27))
#define 	PWR_PCLK_EN()			(RCC->APB1 |= (1<<28))
#define 	DAC_PCLK_EN()			(RCC->APB1 |= (1<<29))




/**********************************************************************************
 * Clock Disable Macro Definitions for various Peripherals of APB1 Bus.
***********************************************************************************/
#define 	TIM2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<0))
#define 	TIM3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<1))
#define 	TIM4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<2))
#define 	TIM5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<3))
#define 	TIM6_PCLK_DI()			(RCC->APB1ENR &= ~(1<<4))
#define 	TIM7_PCLK_DI()			(RCC->APB1ENR &= ~(1<<5))
#define 	TIM12_PCLK_DI()			(RCC->APB1ENR &= ~(1<<6))
#define 	TIM13_PCLK_DI()			(RCC->APB1ENR &= ~(1<<7))
#define 	TIM14_PCLK_DI()			(RCC->APB1ENR &= ~(1<<8))

#define 	WWDG_PCLK_DI()			(RCC->APB1ENR &= ~(1<<11))

#define 	SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1<<14))
#define 	SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1<<15))

#define 	SPDIFRX_PCLK_DI()		(RCC->APB1ENR &= ~(1<<16))

#define 	USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define 	USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))

#define 	UART4_PCLK_DI()			(RCC->APB1 &= ~(1<<19))
#define 	UART5_PCLK_DI()			(RCC->APB1 &= ~(1<<20))

#define 	I2C1_PCLK_DI()			(RCC->APB1 &= ~(1<<21))
#define 	I2C2_PCLK_DI()			(RCC->APB1 &= ~(1<<22))
#define 	I2C3_PCLK_DI()			(RCC->APB1 &= ~(1<<23))

#define 	FMPI2C1_PCLK_DI()		(RCC->APB1 &= ~(1<<24))

#define 	CAN1_PCLK_DI()			(RCC->APB1 &= ~(1<<25))
#define 	CAN2_PCLK_DI()			(RCC->APB1 &= ~(1<<26))

#define 	CEC_PCLK_DI()			(RCC->APB1 &= ~(1<<27))
#define 	PWR_PCLK_DI()			(RCC->APB1 &= ~(1<<28))
#define 	DAC_PCLK_DI()			(RCC->APB1 &= ~(1<<29))




#endif /* INC_STM32F446XX_H_ */
