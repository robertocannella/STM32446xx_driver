# STM32446xx_driver
Drivers for STM32446xx

![STM32F446re-pinout](Documents/SMT32F446re-PinOut.png?raw=true "Title")

This driver was developed specifically for STM43446re Nucleo Board. 

# How to use this Driver: 
# GPIO API Samples
## 001LedToggle:


### Use: Blink onboard LED

The STM32F446re Nucleo Board has onboard green LED (LED2) connected to Port A5 as
show in the documentation:
![STM32F446re-extenstion-connectors](Documents/STM32F446re-ExtConn.png?raw=true "Title")

To configure a General Purpose Input/Output Port, first we must create an instance of ```GPIO_Handle_t``` Here we name the handle GPIOLed.  Using this handle, we can configure the specfic pin and initialize it.  

```
    GPIO_Handle_t GpioLed;
```

The ```GPIO_Handle_t``` structure contains types as well.  The first is the ```GPIO_RegDef_t``` strucutre type.  We use this type to specify the base address of the port we are interesting in reading/writing to.  For this project, since the Green LED is connected to GPIO Port A, Pin 5,  we'll set the GPIOx property to point to GPIOA.
```
    GpioLed.pGPIOx = GPIOA;
```
Now our handle has the right port associated with it.  Let's begin configuring the attributes of our pin.  To do this we use the next ```GPIO_Handle_t``` structure type: ```GPIO_PinConfig_t```.  Let's set the pin number so we can change it's attributes.
```
GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_5;
```






