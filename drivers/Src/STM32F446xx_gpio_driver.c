/*
 * STM32F446xx_gpio_driver.c
 *
 *  Created on: Jun 16, 2022
 *      Author: robertocannella
 */

#include "STM32F446xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}

}
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Enable and Configure the pin
 *
 * @param[in]         - Pin configuration structure for the GPIO
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - To locate which register to apply the pin mode to, we multiply the PinNumber supplied
 *                      by 2 as each GPIO Pin Mode takes up two bits.  For example, If the user passes in the
 *                      PinNumber as GPIO_PIN_6, and a PinMode of GPIO_MODE_ANALOG, the math computes as:
 *
 *                      uint32_t temp = 0; --> 0000000000000000
 *                                             0000000000000000
 *
 *                                                              0000000000000000      000000000000000
 *                      temp = (3 << (2 * 6) --> (3 << 12)  --> 0000000000000011  --> 001100000000000
 *                                                                             ^         ^
 *                      register |= temp	// BITWISE OR to set
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t temp = 0;
	// configure the mode input, output, AF, analog (takes 2 bits)
	if (pGPIO_Handle->GPIOPinConfig.GPIO_PinMode  <= GPIO_MODE_ANALOG) // NOT interrupt mode
	{
		temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= temp;

	}else {	// Interrupt Mode

	}

	// configure the speed (takes 2 bits)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDER |= temp;

	// configure the pull up pull down (takes 2 bits)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinPuPdCtrl << (2 * pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR |= temp;

	// output type (takes 1 bit)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinOutPutTypeCtrl << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= temp;

	// alt function
	if (pGPIO_Handle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		// configure alternate function registers
	}

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Send register back to reset state
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read value of input pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	return 0;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Read all 16 bits of Port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return 0;

}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write value to selected pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number
 * @param[in]         - GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value ){
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write 16 bit value to port
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle the value at the selected pin
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
}
/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Interrupt configuration
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
// IRQ/ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){
}
/*********************************************************************
 * @fn      		  - GPIO_InterruptHandling
 *
 * @brief             - Handler for interrupt trigger condition
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_InterruptHandling(uint8_t PinNumber){
}




