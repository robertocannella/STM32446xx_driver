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



	/*
	 * The default state of GPIO Pin is Floating (HI-z) High impedance.  Not ideal
	 * Either pull up or pull down.
	 *
	 *         Internal Pull UP
	 *
	 *             +  Vcc
	 *             |
	 *             z R1
	 *             Z
	 *   input     |   _|_
	 *    ----<]------[___]--* pin
	 *   buffer         |
	 *
	 *
	 *

	 *
	 *
	 *         Internal Pull Down
	 *
	 *
	 *   input         _|_
	 *    ----<]------[___]--* pin
	 *   buffer    |    |
	 *             Z
	 *             z R2
	 *             |
	 *            ___
	 *             -
	 *
	 *
	 *         Internal Pull up with Open Drain output type
	 *         Writing 1 will drive led up through resistor
	 *         Writing 0 will drive led down to ground
	 *
	 *                 * internal pull-up 10k-50k Ohm (this model)
	 *                 |
	 *                 z R1
	 *                 Z    _   pin     LED
	 *                _|---[_]-*-----[> ----
	 *    0/1       _|                \\   |
	 *    ---[>*---[__                     |
	 *     invert    |                     |
	 *     buffer    |                     |
	 *               |                    ___
	 *              ___                    -
	 *               -
	 *
	 *        Push-Pull configuration does not require any pull-up/pull-down resistor
	 *
	 *                       + Vcc
	 *                       |
	 *            inverted __|
	 *           |-----*[ |__
	 *    0/1    |           |      __       LED
	 *    --[>*--|           |-----[__]-*---[>-----
	 *   invert  |         __|               \\   |
	 *   buffer  |------[ |__                     Z
	 *                       |                    z  Current limiting Resistor
	 *                       |                    |
	 *                      ___                  ___
	 *                       -                    -
	 *
	 */

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_OP_SPD_FAST;
	GpioLed.GPIOPinConfig.GPIO_PinOutPutTypeCtrl = GPIO_OP_TYPE_OD;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while (1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}

	return 0;
}
