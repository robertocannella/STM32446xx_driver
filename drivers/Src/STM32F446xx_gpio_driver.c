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

		// clear and set
		pGPIO_Handle->pGPIOx->MODER &= ~( 0x3 << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber );
		pGPIO_Handle->pGPIOx->MODER |= temp;

	}else {	// Interrupt Mode

	}

	// configure the speed (takes 2 bits)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->OSPEEDER |= temp;

	// configure the pull up pull down (takes 2 bits)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinPuPdCtrl << (2 * pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber));
	// clear and set
	pGPIO_Handle->pGPIOx->PUPDR &= ~( 0x3 << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->PUPDR |= temp;

	// output type (takes 1 bit)
	temp = 0;
	temp = (pGPIO_Handle->GPIOPinConfig.GPIO_PinOutPutTypeCtrl << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber);
	// clear and set
	pGPIO_Handle->pGPIOx->OTYPER &= ~( 0x1 << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->OTYPER |= temp;

	// alt function (takes 4 bits in 2 different registers)  AFR[0] AFR[1]
	if (pGPIO_Handle->GPIOPinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		// Step 1 Logic : divide the PinNumber by 8 to determine which register to use
		//      PinNumber = 9 --> 9/8 = 1 --> Use AFR[1]
		//      PinNumber = 3 --> 3/8 = 0 --> Use AFR[0]

		// Step 2 Logic : use Modulus operator to determine which pin set to use:
		//		PinNumber = 9 --> 9%8 = 1 --> AFR[1] = GPIO_PinAltFuncMode value << (1 * 4) (4 bits)
		//		PinNumber = 3 --> 3%8 = 3 --> AFR[0] = GPIO_PinAltFuncMode value << (3 * 4) (4 bits)

		uint8_t temp1, temp2;

		temp1 = pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber % 8;
		// clear and set
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~( 0xF << pGPIO_Handle->GPIOPinConfig.GPIO_PinNumber );
		pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIOPinConfig.GPIO_PinAltFuncMode << temp2 * 4 );

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

	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}

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
 * @return            - 0 or 1
 *
 * @Note              -
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	// Logic: Shift the requested pin to the right most position (LSB)
	//        Mask the all except the first bit
	uint8_t value;
	value = (uint8_t)( pGPIOx->IDR  >> PinNumber ) & 0x00000001;
	return value;
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
 * @return            - a 16 bit representation of the data at the port
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	// Logic: Shift the requested pin to the right most position (LSB)
	//        Mask the all except the first bit
	uint16_t value;
	value = (uint16_t)( pGPIOx->IDR);
	return value;

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

	if (value == 0){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
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

	pGPIOx->ODR = value;
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




