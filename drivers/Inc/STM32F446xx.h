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

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U				// 112kb
#define SRAM2_BASEADDR						0x2001C000U				// 16kb
#define ROM_BASEADDR						0x1FFF0000U				// system memory
#define SRAM								SRAM1_BASE_ADDR			// main ram

#endif /* INC_STM32F446XX_H_ */
