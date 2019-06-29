#!/bin/bash

avrdude -c avrispv2 -P /dev/tty.usbserial -pt84 -U eeprom:r:eeprom.bin:r