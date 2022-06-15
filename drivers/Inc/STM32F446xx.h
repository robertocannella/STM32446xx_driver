/*
 * STM32F446xx.h
 *
 *  Created on: Jun 14, 2022
 *      Author: robertocannella
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>
/*
 * Base Address of Flash and SRAM Memory (page 62)
 */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U				// 112kb
#define SRAM2_BASEADDR				0x2001C000U				// 16kb
#define ROM_BASEADDR				0x1FFF0000U				// system memory
#define SRAM          SRAM1_BASE_ADDR			// main ram

/*
 * Peripheral Base Address  (pages 57-58)
 */

#define PERIPH_BASEADDR				0x40000000U				// Start of all Bus Addresses
#define APB1_BASEADDR				PERIPH_BASEADDR			// Offset 0x0     		LOW Speed
#define APB2_BASEADDR				0x40010000U				// Offset 0x10000U		LOW Speed
#define AHB1_BASEADDR				0x40020000U				// Offset 0x20000U		HIGH Speed
#define AHB2_BASEADDR				0x50000000U				// Offset 0x10000000	HIGH Speed
#define AHB3_BASEADDR				0xA0001000U				// Offset 0xA0001000	HIGH Speed

/*
 * AHB1 Bus Peripherals (HIGH SPEED BUS)	(page 58)
 */

#define GPIOA_BASEADDR			AHB1_BASEADDR + 0x0000		// Base address of GPIOA Register
#define GPIOB_BASEADDR			AHB1_BASEADDR + 0x0400		// Base address of GPIOB Register
#define GPIOC_BASEADDR			AHB1_BASEADDR + 0x0800		// Base address of GPIOC Register
#define GPIOD_BASEADDR			AHB1_BASEADDR + 0x0C00		// Base address of GPIOD Register
#define GPIOE_BASEADDR			AHB1_BASEADDR + 0x1000		// Base address of GPIOE Register
#define GPIOF_BASEADDR			AHB1_BASEADDR + 0x1400		// Base address of GPIOF Register
#define GPIOG_BASEADDR			AHB1_BASEADDR + 0x1800		// Base address of GPIOG Register
#define GPIOH_BASEADDR			AHB1_BASEADDR + 0x1C00		// Base address of GPIOH Register

#define CRC_BASEADDR				0x40023000U				// Cyclic Redundancy Check Calculation Unit
#define RCC_BASEADDR				0x40023800U				// Reset Clock Control
#define FLASHIR_BASEADDR			0x40023C00U				// Flash Interface Register
#define BKPSRAM_BASEADDR			0x40024000U				// Backup Internal Low Power Memory
#define DMA1_BASEADDR				0x40026000U				// Direct Memory Access Controller 1
#define DMA2_BASEADDR				0x40026400U				// Direct Memory Access Controller 2
#define USBOTGHS_BASEADDR			0x40040000U				// USB 480MPBs

#endif /* INC_STM32F446XX_H_ */
