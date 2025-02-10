# DC-Motor Control Using Atmega88

## Context  
- Course in Applied Mechatronics  

## Repository Includes  
- AVR code for programming microcontroller Atmega88  
- Paired PC code for serial communication (for sending user commands to AVR through PC terminal)  
- Corresponding C-file for `serialport.h` has been omitted from this repository.  

## Features  
- DC motor control in range 0-120 RPM, single direction  
- User interaction options including setting speed, reading speed, etc.  
- PI-control of PWM duty cycle  
- Calculating speed from incremental encoder signals using interrupts  
- Fine-tuning of motor speed using AD-converter and potentiometer  
- Fixed-point arithmetic  