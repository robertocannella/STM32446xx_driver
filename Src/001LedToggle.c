/*
 * 001LedToggle.c
 *
 *  Created on: Jun 17, 2022
 *      Author: robertocannella
 *
 *      LED 2 is connected to PA5
 *
 *
 */

#include "STM32F446xx.h"


void delay(void){
	for (uint32_t i=0; i< 500000; i++);
}


int main(void){



	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioLed.GPIOPinConfig.GPIO_PinOutPutTypeCtrl = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while (1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}

	return 0;
}
