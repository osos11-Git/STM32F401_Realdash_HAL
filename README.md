# STM32F401_Realdash_HAL
STM32F401CCU6 Blackpill 

Base on Arduino... https://github.com/janimm/RealDash-extras/tree/master/RealDash-CAN/Arduino-examples/RealDash_CAN_2way

I'm used CubeIDE 1.7.0 and F4 FW 1.26.2

HAL

USB VCP (CDC) Interrupt Mode, Priority 0 , I think data loss during transfer is not possible.

!! usbd_cdc_if.c file edited !! (line 99-100 and 270-273)

2WAY without anologWrite

Pinouts:

PC13 Pull-Up connected LED and configured on CubeMx

![alt text](https://github.com/osos11-Git/STM32F401_Realdash_HAL/blob/main/F401_RealDash_deneme1/f4_pinout.JPG?raw=true)



