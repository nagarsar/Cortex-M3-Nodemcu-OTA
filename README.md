## Purpose

This project is the continuation of a forum's thread : https://forum.pjrc.com/threads/29607-Over-the-air-updates
The "Flasher" library provided have been updated in order to flash not only through serial communication but also through CAN (Flexray) communication.
The [web app](http://nathanhue.com/ota) developed for this purpose alllow to re-edit and to compile the code quickly and to push the generated firmware to the board through a shield wifi interface PCB custom nodemcu and with a tablet/smartphone Qt application.


## Platform1 : Cortex-M3 
##### Main code:
- MX
##### Library developed :
- Flasher
- FlexCAN
- Metro	
- PWM_Driver3	
- Timer

## Platform2 : NodeMcu 
##### Main code:
- client.ino
##### Library developed :
- esp8266FTPServer



##### Schematic
http://asciiflow.com/#1RKD7Iq99i5qp7UEllrv9KYQFjScWFYXI`
