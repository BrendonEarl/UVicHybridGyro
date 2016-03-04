/*===================
 *=     TO DO       =
 *===================
 *
 *1. Find BaudRate of CAN system
 *Determine id
 *Determine what frame is (CAN library)
 *
*/


//===============================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "I2Cdev.h"     //I2C communication library
#include "MPU6050_9Axis_MotionApps41.h"   //MPU6050 function library
#include "mcp_can.h"    //CAN communication library

#define CAN_ID 0x150

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu;
MCP_CAN CAN(9);


#define OUTPUT_READABLE_WORLDACCEL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// CAN communication vars
int8_t hiDir;
int8_t loDir;
int8_t hiX;
int8_t loX;
int8_t hiY;
int8_t loY;
int8_t hiZ;
int8_t loZ;

unsigned long lastTime = millis(); 

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-2265);
    mpu.setYAccelOffset(-585);
    mpu.setZAccelOffset(2110);
    mpu.setXGyroOffset(50);
    mpu.setYGyroOffset(12);
    mpu.setZGyroOffset(25);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // init can bus, baudrate: 500k
    if(CAN.begin(CAN_500KBPS) == CAN_OK) Serial.print("can init ok!!\r\n");
    else Serial.print("Can init fail!!\r\n");

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // set accelerometer sensitivity
    mpu.setFullScaleAccelRange(1); // +- 4g
    //mpu.setFullScaleGyroRange(0); // 0 is default
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        //Serial.println("Waiting for MPU interrupt or extra packets");
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //Grab/Process Data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //Print Data to Serial bus (For Debuggin):
        //FIFO buffer size counter
        Serial.print(fifoCount);
        Serial.print("\t");
        Serial.print(mpu.getFullScaleAccelRange());
        Serial.print("\t");
        
        //yaw (Direction)/pitch/roll
        Serial.print(ypr[0]);
        Serial.print(",\t");
        Serial.print(ypr[1]);
        Serial.print(",\t");
        Serial.print(ypr[2]);
        Serial.print("\t");

        //Quaternerion Coordinates
        Serial.print(q.w);
        Serial.print(",\t");
        Serial.print(q.x);
        Serial.print(",\t");
        Serial.print(q.y);
        Serial.print(",\t");
        Serial.print(q.z);
        Serial.print("\t");

        //accelerations output (straight from MPU6050)
        Serial.print(aa.x);
        Serial.print(",\t");
        Serial.print(aa.y);
        Serial.print(",\t");
        Serial.print(aa.z);
        Serial.print("\t");

        //gravity components (unit vector)
        Serial.print(gravity.x);
        Serial.print(",\t");
        Serial.print(gravity.y);
        Serial.print(",\t");
        Serial.print(gravity.z);
        Serial.print("\t");

        //acceleartion with gravity removed
        Serial.print(aaReal.x);
        Serial.print(",\t");
        Serial.print(aaReal.y);
        Serial.print(",\t");
        Serial.print(aaReal.z);
        Serial.print("\n");
        
        //Prep data to send over CAN
        ypr[0] = map(ypr[0]*100,-M_PI*100, M_PI*100, 0, 65535);
        aaReal.x = map(aaReal.x, -32768, 32768, 0, 65535);
        aaReal.y = map(aaReal.y, -32768, 32768, 0, 65535);
        aaReal.z = map(aaReal.z, -32768, 32768, 0, 65535);


        //separates value into two 8 bit sets for transmission by typecasting as an integer and shifting and bitwise &ing into 8bit int vars
        //break down Direction data
        unsigned char hiDir = ((((unsigned int) (ypr[0])) >> 8) & 0xff);
        unsigned char loDir = ((((unsigned int) (ypr[0])) >> 0) & 0xff);
        //break down X data
        unsigned char hiX = ((((unsigned int) (aaReal.x)) >> 8) & 0xff);
        unsigned char loX = ((((unsigned int) (aaReal.x)) >> 0) & 0xff);
        //break down Y data
        unsigned char hiY = ((((unsigned int) (aaReal.y)) >> 8) & 0xff);
        unsigned char loY = ((((unsigned int) (aaReal.y)) >> 0) & 0xff);
        //break down Z data
        unsigned char hiZ = ((((unsigned int) (aaReal.z)) >> 8) & 0xff);
        unsigned char loZ = ((((unsigned int) (aaReal.z)) >> 0) & 0xff);

        //Create package to send over CAN
        unsigned char CANbuf[8] = {hiDir, loDir, hiX, loX, hiY, loY, hiZ, loZ};

        //Send data over CAN
        if (millis() - lastTime > 50) {
          lastTime = millis();
          CAN.sendMsgBuf(CAN_ID, 0, 8, CANbuf); //send out the package above to the bus and tell other devices this is a standard frame from 0x00.
        } else {
          Serial.println("No CAN message sent - time limit not exceded (this is okay)");
        }

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
