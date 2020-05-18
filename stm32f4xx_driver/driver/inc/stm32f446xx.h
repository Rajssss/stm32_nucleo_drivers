
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

#define 	SET			 	1
#define 	RESET	 	 	0
#define 	HIGH	 	 	SET
#define 	LOW		 	 	RESET
#define 	ENABLE		 	SET
#define		DISABLE		 	RESET
#define		FLAG_SET		SET
#define		FLAG_RESET		RESET



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
 * GPIO Pin macros.
***********************************************************************************/
#define 	NVIC_IRQ_PRIO_0						0x0
#define 	NVIC_IRQ_PRIO_1						0x1
#define 	NVIC_IRQ_PRIO_2						0x2
#define 	NVIC_IRQ_PRIO_3						0x3
#define 	NVIC_IRQ_PRIO_4						0x4
#define 	NVIC_IRQ_PRIO_5						0x5
#define 	NVIC_IRQ_PRIO_6						0x6
#define 	NVIC_IRQ_PRIO_7						0x7
#define 	NVIC_IRQ_PRIO_8						0x8
#define 	NVIC_IRQ_PRIO_9						0x9
#define 	NVIC_IRQ_PRIO_10					0xA
#define 	NVIC_IRQ_PRIO_11					0xB
#define 	NVIC_IRQ_PRIO_12					0xC
#define 	NVIC_IRQ_PRIO_13					0xD
#define 	NVIC_IRQ_PRIO_14					0xE
#define 	NVIC_IRQ_PRIO_15					0xF





/**********************************************************************************
 * Peripheral Definition (Peripheral base address typecasted to xxxx_Reg_Def_ty).
***********************************************************************************/
#define		GPIOA				((GPIOx_RegDef_ty* )GPIOA_BASEADDR)			//0x4002 0000
#define		GPIOB				((GPIOx_RegDef_ty* )GPIOB_BASEADDR)			//0x4002 0400
#define		GPIOC				((GPIOx_RegDef_ty* )GPIOC_BASEADDR) 		//0x4002 0800
#define		GPIOD				((GPIOx_RegDef_ty* )GPIOD_BASEADDR)			//0x4002 0C00
#define		GPIOE				((GPIOx_RegDef_ty* )GPIOE_BASEADDR)			//0x4002 1000
#define		GPIOF				((GPIOx_RegDef_ty* )GPIOF_BASEADDR)			//0x4002 1400
#define		GPIOG				((GPIOx_RegDef_ty* )GPIOG_BASEADDR)			//0x4002 1800
#define		GPIOH				((GPIOx_RegDef_ty* )GPIOH_BASEADDR)			//0x4002 1C00

#define     RCC    				((RCC_RegDef_ty* )RCC_BASEADDR)

#define		EXTI				((EXTI_RegDef_ty* )EXTI_BASEADDR)

#define 	SYSCFG				((SYSCFG_RegDef_ty* )SYSCFG_BASEADDR)

#define 	SPI1				((SPIx_RegDef_ty* )SPI1_BASEADDR)
#define 	SPI2				((SPIx_RegDef_ty* )SPI2_I2S2_BASEADDR)
#define 	SPI3				((SPIx_RegDef_ty* )SPI3_I2S3_BASEADDR)
#define 	SPI4				((SPIx_RegDef_ty* )SPI4_BASEADDR)




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
 * Registers Reset Macro Definitions for GPIOx Peripherals
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
#define 	TIM14_PCLK_EN()			(RCC->APB1ENR |= (1<<8))

#define 	WWDG_PCLK_EN()			(RCC->APB1ENR |= (1<<11))

#define 	SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define 	SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))

#define 	SPDIFRX_PCLK_EN()		(RCC->APB1ENR |= (1<<16))

#define 	USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define 	USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define 	UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))
#define 	UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))

#define 	I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define 	I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define 	I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))

#define 	FMPI2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<24))

#define 	CAN1_PCLK_EN()			(RCC->APB1ENR |= (1<<25))
#define 	CAN2_PCLK_EN()			(RCC->APB1ENR |= (1<<26))

#define 	CEC_PCLK_EN()			(RCC->APB1ENR |= (1<<27))
#define 	PWR_PCLK_EN()			(RCC->APB1ENR |= (1<<28))
#define 	DAC_PCLK_EN()			(RCC->APB1ENR |= (1<<29))




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





/**********************************************************************************
 * Clock Enable Macro Definitions for various Peripherals of APB2 Bus.
***********************************************************************************/
#define 	TIM1_PCLK_EN()			(RCC->APB2ENR |= (1<<0))
#define 	TIM8_PCLK_EN()			(RCC->APB2ENR |= (1<<1))
#define 	TIM9_PCLK_EN()			(RCC->APB2ENR |= (1<<16))
#define 	TIM10_PCLK_EN()			(RCC->APB2ENR |= (1<<17))
#define 	TIM11_PCLK_EN()			(RCC->APB2ENR |= (1<<18))

#define 	USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define 	USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))

#define 	ADC1_PCLK_EN()			(RCC->APB2ENR |= (1<<8))
#define 	ADC2_PCLK_EN()			(RCC->APB2ENR |= (1<<9))
#define 	ADC3_PCLK_EN()			(RCC->APB2ENR |= (1<<10))

#define 	SDIO_PCLK_EN()			(RCC->APB2ENR |= (1<<11))

#define 	SPI1_PCLK_EN()			(RCC->APB2ENR |= (1<<12))
#define 	SPI4_PCLK_EN()			(RCC->APB2ENR |= (1<<13))

#define 	SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

#define 	SAI1_PCLK_EN()			(RCC->APB2ENR |= (1<<22))
#define 	SAI2_PCLK_EN()			(RCC->APB2ENR |= (1<<23))





/**********************************************************************************
 * Clock Disable Macro Definitions for various Peripherals of APB2 Bus.
***********************************************************************************/
#define 	TIM1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<0))
#define 	TIM8_PCLK_DI()			(RCC->APB2ENR &= ~(1<<1))
#define 	TIM9_PCLK_DI()			(RCC->APB2ENR &= ~(1<<16))
#define 	TIM10_PCLK_DI()			(RCC->APB2ENR &= ~(1<<17))
#define 	TIM11_PCLK_DI()			(RCC->APB2ENR &= ~(1<<18))

#define 	USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define 	USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

#define 	ADC1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<8))
#define 	ADC2_PCLK_DI()			(RCC->APB2ENR &= ~(1<<9))
#define 	ADC3_PCLK_DI()			(RCC->APB2ENR &= ~(1<<10))

#define 	SDIO_PCLK_DI()			(RCC->APB2ENR &= ~(1<<11))

#define 	SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<12))
#define 	SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1<<13))

#define 	SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

#define 	SAI1_PCLK_DI()			(RCC->APB2ENR &= ~(1<<22))
#define 	SAI2_PCLK_DI()			(RCC->APB2ENR &= ~(1<<23))





/**********************************************************************************
 * Returns a PORTCODE(1 - 7) based on the base address of the port
***********************************************************************************/
#define 	GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
												(x == GPIOB) ? 1:\
												(x == GPIOC) ? 2:\
												(x == GPIOD) ? 3:\
												(x == GPIOE) ? 4:\
												(x == GPIOF) ? 5:\
												(x == GPIOG) ? 6:\
												(x == GPIOH) ? 7:0)






/**********************************************************************************
 * IRQ(Interrupt Request) Number Macros for EXTI Lines
***********************************************************************************/
#define 	IRQ_NO_EXTI0					6
#define 	IRQ_NO_EXTI1					7
#define 	IRQ_NO_EXTI2					8
#define 	IRQ_NO_EXTI3					9
#define 	IRQ_NO_EXTI4					10
#define 	IRQ_NO_EXTI9_5					23
#define 	IRQ_NO_EXTI15_10				40





/**********************************************************************************
 * NVIC ISERx Address Macros
***********************************************************************************/
#define 	NVIC_ISER0						((__VOL uint32_t* ) 0xE000E100)
#define 	NVIC_ISER1						((__VOL uint32_t* ) 0xE000E104)
#define 	NVIC_ISER2						((__VOL uint32_t* ) 0xE000E108)
#define 	NVIC_ISER3						((__VOL uint32_t* ) 0xE000E10C)
#define 	NVIC_ISER4						((__VOL uint32_t* ) 0xE000E110)
#define 	NVIC_ISER5						((__VOL uint32_t* ) 0xE000E114)
#define 	NVIC_ISER6						((__VOL uint32_t* ) 0xE000E118)
#define 	NVIC_ISER7						((__VOL uint32_t* ) 0xE000E11C)



/**********************************************************************************
 * NVIC ICERx Address Macros
***********************************************************************************/
#define 	NVIC_ICER0						((__VOL uint32_t* ) 0XE000E180)
#define 	NVIC_ICER1						((__VOL uint32_t* ) 0XE000E184)
#define 	NVIC_ICER2						((__VOL uint32_t* ) 0XE000E188)
#define 	NVIC_ICER3						((__VOL uint32_t* ) 0XE000E18C)
#define 	NVIC_ICER4						((__VOL uint32_t* ) 0XE000E190)
#define 	NVIC_ICER5						((__VOL uint32_t* ) 0XE000E194)
#define 	NVIC_ICER6						((__VOL uint32_t* ) 0XE000E198)
#define 	NVIC_ICER7						((__VOL uint32_t* ) 0XE000E19C)



/**********************************************************************************
 * NVIC Interrupt Priority Register Base Address Macros
***********************************************************************************/
#define 	NVIC_PR_BASE_ADDR				((__VOL uint32_t* ) 0xE000E400)

//Number of Priority Register Bits implemented in STM32F446RE is 4
#define 	NVIC_NO_PR_BITS_IMPLEMENTED			4






/**********************************************************************************
 * NVIC Interrupt Priority value macros
***********************************************************************************/
#define 	GPIO_PIN_NO_0						0x0
#define 	GPIO_PIN_NO_1						0x1
#define 	GPIO_PIN_NO_2						0x2
#define 	GPIO_PIN_NO_3						0x3
#define 	GPIO_PIN_NO_4						0x4
#define 	GPIO_PIN_NO_5						0x5
#define 	GPIO_PIN_NO_6						0x6
#define 	GPIO_PIN_NO_7						0x7
#define 	GPIO_PIN_NO_8						0x8
#define 	GPIO_PIN_NO_9						0x9
#define 	GPIO_PIN_NO_10						0xA
#define 	GPIO_PIN_NO_11						0xB
#define 	GPIO_PIN_NO_12						0xC
#define 	GPIO_PIN_NO_13						0xD
#define 	GPIO_PIN_NO_14						0xE
#define 	GPIO_PIN_NO_15						0xF





/**********************************************************************************
 * SPIx Peripheral Control Register (CR1) Bitfields
***********************************************************************************/
#define 	SPI_CR1_CPHA						0x0			//Clock phase
#define 	SPI_CR1_CPOL						0x1			//Clock polarity
#define 	SPI_CR1_MSTR						0x2			//Master selection
#define 	SPI_CR1_BR							0x3			//Baud rate control (SCLK)
#define 	SPI_CR1_SPE							0x6			//SPI enable
#define 	SPI_CR1_LSBFIRST					0x7			//Frame format
#define 	SPI_CR1_SSL							0x8			//Internal slave select
#define 	SPI_CR1_SSM							0x9			//Software slave management
#define 	SPI_CR1_RXONLY						0xA			//Receive only mode enable
#define 	SPI_CR1_DFF							0xB			//Data frame format
#define 	SPI_CR1_CRCNEXT						0xC			//CRC transfer next
#define 	SPI_CR1_CRCEN						0xD			//Hardware CRC calculation enable
#define 	SPI_CR1_BIDIOE						0xE			//Output enable in bidirectional mode
#define 	SPI_CR1_BIDIMODE					0xF			//Bidirectional data mode enable




/**********************************************************************************
 * SPIx Peripheral Control Register (CR2) Bitfields
***********************************************************************************/
#define 	SPI_CR2_RXDMAEN						0
#define 	SPI_CR2_TXDMAEN						1
#define 	SPI_CR2_SSOE						2
#define 	SPI_CR2_FRF							4
#define 	SPI_CR2_ERRIE						5
#define 	SPI_CR2_RXNEIE						6
#define 	SPI_CR2_TXNEIE						7



/**********************************************************************************
 * SPIx Peripheral Status Register (SR) Bitfields
***********************************************************************************/
#define 	SPI_SR_RXNE							0
#define 	SPI_SR_TXE							1
#define 	SPI_SR_CHSIDE						2
#define 	SPI_SR_UDR							3
#define 	SPI_SR_CRCERR						4
#define 	SPI_SR_MODF							5
#define 	SPI_SR_OVR							6
#define 	SPI_SR_BSY							7
#define 	SPI_SR_FRE							8





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
 * External Interrupts (EXTI) Registers Structure
***********************************************************************************/
typedef struct
{
	__VOL uint32_t IMR;						//Interrupt mask register
	__VOL uint32_t EMR;						//Event mask register
	__VOL uint32_t RTSR;					//Rising trigger selection register
	__VOL uint32_t FTSR;					//Falling trigger selection register
	__VOL uint32_t SWIER;					//Software interrupt event register
	__VOL uint32_t PR;						//Pending register

}EXTI_RegDef_ty;





/**********************************************************************************
 * System Configuration (SYSCFG) Registers Structure
***********************************************************************************/
typedef struct
{
	__VOL uint32_t MEMRMP;					//SYSCFG memory remap register
	__VOL uint32_t PMC;						//SYSCFG peripheral mode configuration register
	__VOL uint32_t EXTICR[4];				//SYSCFG external interrupt configuration register 1
		  uint32_t RESERVED1[2];			//Reserved memory regions
	__VOL uint32_t CMPCR;					//Compensation cell control register
		  uint32_t RESERVED2[2];			//Reserved memory regions
	__VOL uint32_t CFGR;					//SYSCFG configuration register

}SYSCFG_RegDef_ty;






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
 * SPI Peripheral Registers Structure
***********************************************************************************/
typedef struct
{
	__VOL uint32_t CR1;						//SPIx control register 1
	__VOL uint32_t CR2;						//SPIx control register 2
	__VOL uint32_t SR;						//SPIx status register
	__VOL uint32_t DR;						//SPIx data register
	__VOL uint32_t CRCPR;					//SPIx CRC polynomial register
	__VOL uint32_t RXCRCPR;					//SPIx RX CRC register
	__VOL uint32_t TXCRCR;					//SPIx TX CRC register
	__VOL uint32_t I2SCFGR;					//SPIx_I2S configuration register
	__VOL uint32_t I2SPR;					//SPIx I2S prescaler register

}SPIx_RegDef_ty;




#endif /* INC_STM32F446XX_H_ */
