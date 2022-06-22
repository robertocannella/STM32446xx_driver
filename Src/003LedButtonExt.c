/*
 * 002LedButton.c
 *
 *  Created on: Jun 19, 2022
 *      Author: robertocannella
 */


/* See Tinker Cad for Setup */

/*
 *   OPERATION:
 *   	External button operates an external LED.
 *
 *   	5V  from STM3244Rxx to Breadboard 5v
 *   	GND from STM3244rxx to Breadboard GND
 *
 *
 *   	One side of Button is wired to port PB8 ( PD15 Adruino )
 *   		- A pullup resistor is required to from 5v to Button (enabled in software)
 *   	Other side of Button is connected to GND
 *
 *   	The anode of the Led is connected to PA1 (PA1 on Adruino)
 *   	The cathod of the Led is connected to a 220 OHM  Resistor and then routed to GND
 *
 *   	GPIOA Port 1 Output Configuration:
 *   	PinNumber:  1
 *   	PinMode:    OUT
 *   	PinSpeed:   FAST
 *   	OutputType: PUSH/PULL
 *   	PUPD:       NO PUPD
 *
 *   	GPIOB Port 8 Input Configuration:
 *   	PinNumber:  8
 *   	PinMode:    IN
 *   	Speed:      FAST
 *   	PUPD:		Pull Up (keeps pin out of H-Z state)
 *
 *
 *
 */

#include "STM32F446xx.h"
#define LOW 0
#define HIGH 1
#define BTN_PRESSED LOW

void delay(void){
	for (uint32_t i=0; i< 500000/2; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBtn;


	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioLed.GPIOPinConfig.GPIO_PinOutPutTypeCtrl = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;


	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GpioBtn.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioBtn.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;


	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_PeriClkCtrl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	while (1){
		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_8) == BTN_PRESSED){
			delay(); //Avoid denouncing
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_1);
		}
	}

	return 0;
}
