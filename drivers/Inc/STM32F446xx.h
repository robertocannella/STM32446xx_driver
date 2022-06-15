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
#define SRAM1_BASEADDR				0x20000000U			// 112kb
#define SRAM2_BASEADDR				0x2001C000U			// 16kb
#define ROM_BASEADDR				0x1FFF0000U			// system memory
#define SRAM						SRAM1_BASE_ADDR		// main ram

/*
 * Peripheral Base Address  (pages 57-58)
 */

#define PERIPH_BASEADDR				0x40000000U			// Start of all Bus Addresses
#define APB1_BASEADDR				0x40000000U			// Offset 0x0     		LOW Speed
#define APB2_BASEADDR				0x40010000U			// Offset 0x10000U		LOW Speed
#define AHB1_BASEADDR				0x40020000U			// Offset 0x20000U		HIGH Speed
#define AHB2_BASEADDR				0x50000000U			// Offset 0x10000000	HIGH Speed
#define AHB3_BASEADDR				0xA0001000U			// Offset 0xA0001000	HIGH Speed




#endif /* INC_STM32F446XX_H_ */
