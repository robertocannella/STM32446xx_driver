# STM32446xx_driver
Drivers for STM32446xx

This driver was developed specifically for STM43446re Nucleo Board. 

# How to use this Driver: 
# GPIO API Samples
## 001LedToggle:


### Use: Blink onboard LED

The STM32F446re Nucleo Board has onboard green LED (LED2) connected to Port A5 as
show in the documentation:
![STM32F446re-extenstion-connectors](Documents/STM32F446re-ExtConn.png?raw=true "Title")

To configure a General Purpose Input/Output Port, first we must create an  instance of ```GPIO_Handle_t``` which points to the correct Port.  Here we give name the handle GPIOLed.  Using this handle, we can configure the specfic pin and initialize it.  


![STM32F446re-pinout](Documents/SMT32F446re-PinOut.png?raw=true "Title")



