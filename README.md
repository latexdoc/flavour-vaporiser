# flavour-vaporiser
automatic flavour-vaporiser

## Features
This device has the following features:
* manual flavour delivery on button-press 
* switch between automatic/manual mode
* start and resume timer in automatic mode
* stop timer in automatic mode
* pause of automatic mode
* report output of valve status through LED
     LED off: valve is shut / timer if off 
     LED on:  valve open
     LED blink: valve is shut / timer paused
* report status of the device (heartbeat-LED)
 
## Description of the hardware: 
* output-pins are always HIGH-Logic: HIGH || 1 == HIGH (3.3V) on the Pin
* Pin X -> 4.7kOhm -> LED_A -> LED_K -> GND
* Pin X -> relais-coil -> GND

## License
This project is published under the GPLv3 license. 
 
## Tools used
The schematics were done with Kicad (http://kicad-pcb.org/). A howto can be found under the link as well. The firmware of the device was written via Arduino 1.8.4 (https://www.arduino.cc/).

## Questions and contact
If there're questions feel free to reach out either via mail (latexdox@gmx.de) or via fetlife (https://fetlife.com/users/1444096).

## Finally have fun!
