# Subduino

An Arduino project for Subaru SSM to Can-bus conversion

The code in this project can be used to poll a Subaru WRX / STi ECU, using the Subaru SSM protocol over K-Line. 

Subaru ECU's supposedly support reading blocks of address space, but I've never been able to get it to work. 
Instead, this project polls a small group of addresses / parameters and packages them up into CAN packets.

The hardware I've used in this project is a standard CAN shield, and serial-to-kline adapter chip from MC: the MC33660.
The only hangup I experienced was in wiring the MC33660, don't skip the pull-up resistor!

## SSMCAN Arduino Library

This folder includes the SSM protocol as an Arduino library. Maybe one day I'll get around to formally publishing it.

On MacOS, copy it to ~/Documents/Arduino/libraries/.
