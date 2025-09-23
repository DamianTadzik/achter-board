# CAN NOT

## Description
This is library/wrapper for CAN peripherial. It is based on FreeRTOS and HAL. It is meant to be reliable and user friendly. Naming convention "can not" "cant" is used to distinguish that this is different implementation from "can" libraries used previously (also little humorous because can can't). 

## SuperDescription
This repository should be added as submodule to your project!

API consist of four C functions, usage is described inside header file:
- `cant_main_init()` 
- `cant_freertos_init()`
- `cant_transmit()`
- `cant_receive()`

User also should provide define of one of can instances used, and for now has only three options:
- STM32F446RE `CANT1` or `CANT2` 
- STM32Fxxxx `CAN`

There are debug counters implemented in source file, and they are available after defining:
- `CANT_DEBUG`

## Usage example and required configuration
This section is not done yet as i do not have energy and time for this. If you are smart programmer then you will find usage examples in header file. Configuration could be found in project such as can-logger (this one made by Damian Brzana)

## Created by
Damian Brzana aka @DamianTadzik on github aka @DamianTadziu on gitlab - 2024
