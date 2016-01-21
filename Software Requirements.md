#UVic Formula Hybrid 2016

##Gyro Project

###Introduction

UVic Formula Hybrid is a multi-disciplinary project group at the University of Victoria. Each year UVic and many other teams across North America join to compete. This year, UVic's car is in need of a sensor providing feedback with respect to the car's x,y,z acceleration, as well it's directional characteristics while turning. Therefore allowing the team to gather more data during testing for safety and knowledge purposes.

####Purpose

To provide vehicle direction and x, y, and z acceleration data, allowing the team to better understand the loads which the car undergoes during testing.

####Definitions

CAN: Close Area Network, often used in automobiles

####System overview

The gyro board will be fixed to the car and connected to a CANduino board, which will be required to send data over the CAN system to later be transmitted via the telemetry system.

###Overall description

####System Interfaces

The interfaces of concern are:

* Gyro board to CANduino
* CANduino to CAN system

####Hardware interfaces

Physically the gyro board will be connected to the CANduino with the following pins:

* 3.3V power
* GND
* SCL
* SDA

Each must be connected to pins on the CANduino with matching labels.

####Software interfaces

####Communication Interfaces

As mentioned above in the system interfaces, there are two interfaces. The former (gyro board to CANduino) uses I2C, while the latter (CANduino to CAN system) uses the CAN protocol.

####Memory Constraints

Exact memory size is not known at this time, but should be assumed as very small.

####Operations


####Site Adaptation Requirements

####Product functions

####Constraints, assumptions and dependencies

###Specific requirements

####Functional requirements
