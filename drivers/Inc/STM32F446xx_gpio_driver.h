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

}GPIO_Handle_t;


/*****************************************************************************************
 *
 * 				API Supported by this driver
 *
 */

// Peripheral clock
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);        // Enable the GPIO Peripheral Clock

// Initialize/DeInit
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);                          // Enable and Configure the pin
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);                              // Send register back to reset state

// Read/Write
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);                    // Read value of input pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);                                      // Read all 16 bits of Port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value );		 // Write value to selected pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);                          // Write 16 bit value to port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);                         // Toggle the value at the selected pin

// IRQ/ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);                 // Interrupt configuration
void GPIO_InterruptHandling(uint8_t PinNumber);                                              // Handler for interrupt trigger condition




#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
