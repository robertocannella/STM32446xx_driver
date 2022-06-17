/*
 * STM32F446xx_gpio_driver.h
 *
 *  Created on: Jun 16, 2022
 *      Author: robertocannella
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "STM32F446xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
	uint8_t GPIO_PinOutPutTypeCtrl;
	uint8_t GPIO_PinAltFuncMode;

}GPIO_PinConfig_t;

/*
 * The handle structure for a GPIO pin
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;                 // The base address of the GPIO to which the pin belongs
	GPIO_PinConfig_t GPIOPinConfig;        // The GPIO pin configuration settings

}GPIO_handle_t;


/*****************************************************************************************
 *
 * 				API Supported by this driver
 *
 */

// Peripheral clock
void GPIO_PeriClkCtrl(void);              // Enable the GPIO Peripheral Clock

// Initialize/DeInit
void GPIO_Init(void);
void GPIO_DeInit(void);

// Read/Write
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

// IRQ/ISR Handling
void GPIO_IRQConfig(void);                 // Interrupt configuration
void GPIO_InterruptHandling(void);         // Handler for interrupt trigger condition




#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
