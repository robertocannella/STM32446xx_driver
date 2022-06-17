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
	uint8_t GPIO_PinNumber;               /*! < possible values from @GPIO_PIN_NUMBERS > */
    uint8_t GPIO_PinMode;                 /*! < possible values from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinSpeed;                /*! < possible values from @GPIO_PIN_SPEED > */
	uint8_t GPIO_PinPuPdCtrl;             /*! < possible values from @GPIO_PIN_PUPD > */
	uint8_t GPIO_PinOutPutTypeCtrl;       /*! < possible values from @GPIO_PIN_OUTPUT_TYPES > */
	uint8_t GPIO_PinAltFuncMode;          /*! < possible values from @GPIO_PIN_ALTFN_TYPES > */

}GPIO_PinConfig_t;

/*
 * The handle structure for a GPIO pin
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;                 // The base address of the GPIO to which the pin belongs
	GPIO_PinConfig_t GPIOPinConfig;        // The GPIO pin configuration settings

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_0           0
#define GPIO_PIN_1           1
#define GPIO_PIN_2           2
#define GPIO_PIN_3           3
#define GPIO_PIN_4           4
#define GPIO_PIN_5           5
#define GPIO_PIN_6           6
#define GPIO_PIN_7           7
#define GPIO_PIN_8           8
#define GPIO_PIN_9           9
#define GPIO_PIN_10          10
#define GPIO_PIN_11          11
#define GPIO_PIN_12          12
#define GPIO_PIN_13          13
#define GPIO_PIN_14          14
#define GPIO_PIN_15          15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN         0             // input mode
#define GPIO_MODE_OUT        1             // output mode
#define GPIO_MODE_ALTFN      2             // alternate function mode
#define GPIO_MODE_ANALOG     3             // analog mode
#define GPIO_MODE_IT_FT      4             // external interrupt falling trigger
#define GPIO_MODE_IT_RT      5             // external interrupt rising trigger
#define GPIO_MODE_IT_RFT     6             // external interrupt falling/rising trigger

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP       0            // push/pull type
#define GPIO_OP_TYPE_OD       1            // open drain type

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */

#define GPIO_OP_SPD_LOW       0            // low speed
#define GPIO_OP_SPD_MED       1            // medium speed
#define GPIO_OP_SPD_FAST      2            // fast speed
#define GPIO_OP_SPD_HIGH      3            // high speed


/*
 * @GPIO_PIN_PUPD
 * GPIO  pin possible pull up AND pull down configuration macros
 */

#define GPIO_NO_PUPD          0            // no pull up pull down
#define GPIO_PIN_PU           1            // pull up
#define GPIO_PIN_PD           2            // pull down

/*
 * @GPIO_PIN_ALTFN_TYPES
 * GPIO pin alternate function types
 */

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
