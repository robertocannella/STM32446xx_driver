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
		if (GPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (GPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (GPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (GPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (GPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (GPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (GPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (GPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else
	{
		if (GPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if (GPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if (GPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if (GPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if (GPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (GPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if (GPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if (GPIOx == GPIOH){
			GPIOH_PCLK_DI();
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
 * @Note              -
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
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
 * @brief             - Handler for interrupt trigger conditio
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




