# README #
Motion Sensor using MPU9250 and nRF52840-DK

### Summary ###
This repo contains source code of motion sensor device.
Date: 06/11/2021
-> Includes SDK and Firmware Source COde
-> Firmware to send "Hello World" String on USB COM Port

Date: 06/29/2021
-> Firmware to send sensor data String on USB COM Port

### How do I get set up? ###
Kindly refer below link for environment setup.
https://bitbucket.org/jumpcatbucket/jc_dk/wiki/environment_setup


Serial Protocol document:
https://bitbucket.org/jumpcatbucket/jc_dk/wiki/Simple%20serial%20line%20protocol


### LED Functionality ###
nRF52840-DK board has 4 on board LEDs [LED1, LED2, LED3, LED4]

1. If you have not connected Micro USB for Serial Communication, ALL LEDs will remain OFF.

2. If you will connect Micro USB for Serial Communication, then LED1 will glow Solid

3. If you open HTerm utility with genrated COM port and 115200 baudrate,  then LED2 will glow Solid and LED3 will blink every second on every "Hello World" sent on serial port
