/*
 * 002LedButton.c
 *
 *  Created on: Jun 19, 2022
 *      Author: robertocannella
 */


#include "STM32F446xx.h"
#define LOW 0
#define BTN_PRESSED LOW

void delay(void){
	for (uint32_t i=0; i< 500000/2; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBtn;


	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioLed.GPIOPinConfig.GPIO_PinOutPutTypeCtrl = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;


	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioBtn.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioBtn.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;


	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_PeriClkCtrl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	while (1){
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == BTN_PRESSED){
			delay(); //Avoid denouncing
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		}
	}

	return 0;
}
