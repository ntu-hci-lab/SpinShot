# SpinShot STM32 Control Code

## Overview

This repository contains the control code for the SpinShot STM32 microcontroller. The primary functions of this code are:
1. To facilitate communication between the SpinShot device and a computer via serial ports.
2. To control the motor and the solenoid components of the device.
3. To monitor the angle of the flywheel, ensuring precise operational performance.

## Development Environment

The project leverages STM32CubeMX for graphical rapid configuration of the STM32 chip, automatically generating C code compatible with Keil µVision IDE. The development process involves writing code in Keil µVision 5 IDE and subsequently using an STLink to flash the compiled files onto the control board.

## Getting Started

Official Tutorial
1. Use STM32CubeMX in MDK Projects(https://www.keil.com/pack/doc/STM32Cube/html/cubemx_using.html)

For individuals new to this type of development, aside from various online resources, Robomaster provides detailed guidance on using Keil µVision 5 IDE:
1. Running your first piece of code, exemplified by the [“RoboMaster M2006 Power System DEMO Example Program”](https://github.com/ntu-hci-lab/SpinShot/blob/main/Firmware/Robomaster_M2006_Power_System_Demo_Example_Program.pdf) (available only in Chinese).
2. Robomaster Board C Examples available at [Robomaster Development Board C Examples](https://github.com/RoboMaster/Development-Board-C-Examples/tree/master)

## Important Notes

To open the STM32Cube project, use the `SpinShot.ioc` file. To access the Keil project, open the `MDK_ARM/SpinShot.uvprojx` file.
