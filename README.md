# DTM0660-flasher-arduino-sketch
A sketch to update the 24c02 on a DTM0660 based multimeter to add additional features

The code scans the i2c bus from your STM32F103 based 'duino (will probably work with pretty much any Arduino or clone). 
It then dumps the contents, and attempts to enable RS232, change the meter from 4000 count to 6000 or 8000 count, or perform
whatever other mod you require (additional coding require of course). 

You use this entirely at your own risk. No ifs, no buts, any problems and you are on your own. 
The code is licensed GPL3
