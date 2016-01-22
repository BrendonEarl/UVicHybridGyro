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

Data is transmitted from MPU6050 using I2C into the fifoBuffer. Data is then pulled from the fifoBuffer into __q__ and __aa__ for the quaternion coordinates and acceleration components, respectively. From that gravity and linear acceleration is calculated. ==Issue:== expected values for q is unknown at this time.

####Site Adaptation Requirements

AFS_SEL = 1  
LSB/g = 8192

Data transmitted has had the following operations performed:  
$\times$ 100  
typecast from float to int  
hi = data >> 8 //right shift 8  
lo = data >> 0 //right shift 0  
Both ANDed with 0xff onto 8 bit integers

####Product functions

####Constraints, assumptions and dependencies

###Specific requirements

####Functional requirements


##Understanding The Libraries

####CANBusShield

The CANBusShield library is intended for communication between the Arduino with the CAN shield attached, and the rest of the CAN network.

In particular the send data function will be used:

    CAN.sendMsgBuf(INT8U id, INT8U ext, INT8U len, data_buf);

This is a function to send data onto the bus. In which:

**id** represents where the data come from.

**ext** represents the status of the frame. '0' means standard frame. '1' means extended frame.

**len** represents the length of this frame.

**data_buf** is the content of this message.

For example, In the 'send' example, we have:

~~~
unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

CAN.sendMsgBuf(0x00, 0, 8, stmp); //send out the message 'stmp' to the bus and tell other devices this is a standard frame from 0x00.
~~~


####I2Cdev

The I2Cdev library allows the Arduino to connect to the MPU6050 gyro board via I2C. This library is utilized by the MPU6050 library to read and write bits, bytes, or words to and from the board. This allows one to view the accelerometer and gyroscope data, which is then later processed using the MPU6050 library.

####MPU6050

The MPU6050 library is a custom set of functions for this particular gyro/acceleration board. As was recently discovered, the library is still under construction, so upon using any functions, they should be traced and verified to ensure the expected output is produced.

__The key functions are as follows:__

	void MPU6050::setFullScaleAccelRange(uint8_t range)

Resides in MPU6050.cpp: Sets the precision and range of g's being interpreted. It does this by writing the 8 bit value "range" to the AFS_SEL variable held on the MPU6050. 

| AFS_SEL | Range (g) | LSB/g |
|:-------:|:---------:|:-----:|
|    0    |  $\pm$2   | 16384 |
|    1    |  $\pm$4   | 8192  |
|    2    |  $\pm$6   | 4096  |
|    3    |  $\pm$8   | 2048  |

AFS_SEL is set to 0 by default, but is set to 1 in case the car pulls over 2g.

<br>

	mpu.dmpGetQuaternion(&q, fifoBuffer);

Resides in MPU6050\_6Axis\_MotionApps20: Gets the quaternion coordinate values from the fifoBuffer and stores them in the container __q__ as w, x, y, z float variables.

<br>

	mpu.dmpGetAccel(&aa, fifoBuffer);

Resides in MPU6050\_6Axis\_MotionApps20: Gets the acceleration values from the fifoBuffer and stores them in the variable __aa__ as x, y, z, where the values seem to follow the pattern:

| AFS_SEL | int = #/g |
|:-------:|:---------:|
|    0    |   1020    |
|    1    |   560     |
|    2    |   280     |
|    3    |   140     |	

<br>

	mpu.dmpGetGravity(&gravity, &q);
	
Resides in MPU6050\_6Axis\_MotionApps20: Gets the gravity vector in what seems to be x, y, z components of a unit vector. 

<br>

	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

Resides in MPU6050\_6Axis\_MotionApps20: Gets the Yaw, Pitch, Roll components of the gyro board based on its position in quaternion coordinates relative to the gravity vector and outputs the yaw, pitch, roll components in the __ypr__ array.

<br>

	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	
Resides in MPU6050\_6Axis\_MotionApps20: Gets 'real' acceleration (in __aaReal__) by subtracting the gravity vector in each of the x, y, z components.  
==Issue:== aa and gravity values don't line up (aa is not multiplied by any constant, constant to multiply gravity doesn't change with AFS_SEL - constant value referred to is that found in the LSB/g column in a table above)