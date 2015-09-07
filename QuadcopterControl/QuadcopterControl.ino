#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <helper_3dmath.h>
#include "Wire.h"
#include "I2Cdev.h"
#include <PID_v1.h>
#include <Servo.h>

// Constants ------------------------------------------------------------ |
#define ESC_PIN_1 3
#define ESC_PIN_2 5
#define ESC_PIN_3 6
#define ESC_PIN_4 9

#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000
#define MAX_PID 100
#define MIN_PID -100

// ESC Variables -------------------------------------------------------- |
Servo m1, m2, m3, m4;

// IMU Variables -------------------------------------------------------- |
MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float throttleIn = 1000;
float throttle = 1000;
float m1Throttle;
float m2Throttle;
float m3Throttle;
float m4Throttle;

int indexCommaA;
int indexCommaB;

bool readThrottle = false;
bool readPos = false;

// variables to calculate pid values
float setPitch = 0;
float setRoll = 0;
float setYaw = 0;
float inputPitch = 0;
float inputRoll = 0;
float inputYaw = 0;
float pidOutPitch = 0;
float pidOutRoll = 0;
float pidOutYaw = 0;

// MPU control/status vars


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// tuning parameters
float Kp = 5, Ki = 5, Kd = 1;

// PID objects
PID yawPID(&inputYaw, &pidOutYaw, &setYaw, Kp, Ki, Kd, DIRECT);
PID pitchPID(&inputPitch, &pidOutPitch, &setPitch, Kp, Ki, Kd, DIRECT);
PID rollPID(&inputRoll, &pidOutRoll, &setRoll, Kp, Ki, Kd, DIRECT);

String message = "";
String yawString;
String pitchString;
String rollString;

// ----------------------------------------------------------------------- |
// Setup ----------------------------------------------------------------- |
void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // TWBR = 24; // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
  
  //Setup usb serial connection to computer
  Serial.begin(115200);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
    
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
  
  m1.attach(ESC_PIN_1);
  m2.attach(ESC_PIN_2);
  m3.attach(ESC_PIN_3);
  m4.attach(ESC_PIN_4);

  // Turn PIDs on
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(MIN_PID, MAX_PID);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(MIN_PID, MAX_PID);
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(MIN_PID, MAX_PID);
}

// ----------------------------------------------------------------------- |
// Loop ------------------------------------------------------------------ |
void loop()
{

  // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      
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
        Serial.println();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x01) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180/M_PI);
        //Serial.print("\t");
        //Serial.print(ypr[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180/M_PI);
    }

  //Read from bluetooth and write to usb serial
  if(Serial.available())
  {
    char charIn = (char)Serial.read();
    // Reading a throttle value
    if (charIn == 't') {
      Serial.println("READING THROTTLE VALUE");
      readThrottle = true;
      readPos = false;
    } else if (charIn == 'p') {
      Serial.println("READING POSITION VALUE");
      readThrottle = false;
      readPos = true;
    } else {
      Serial.println("Reading in character : " + (String)charIn);
      if (charIn != 'n') {
        message += charIn;
        Serial.println("inchar : " + message);

      // Reading in throttle values
      } else if (readThrottle == true) {
        throttleIn = atof(message.c_str());
        throttleIn = map(throttleIn, 0, MIN_THROTTLE, 1040, MAX_THROTTLE);
        if (throttleIn > 1030 && throttleIn <= MAX_THROTTLE)
            throttle = throttleIn;
        Serial.println("THROTTLE : " + message);
        message = "";

      // Reading in comma-delimited position values
      } else if (readPos) {
        indexCommaA = message.indexOf(',');
        indexCommaB = message.indexOf(',', indexCommaA + 1);

        yawString = message.substring(0, indexCommaA);
        pitchString = message.substring(indexCommaA + 1, indexCommaB);
        rollString = message.substring(indexCommaB + 1);

        inputYaw = atof(yawString.c_str());
        inputPitch = atof(pitchString.c_str());
        inputRoll = atof(rollString.c_str());

        Serial.println("YAW : " + yawString);
        Serial.println("PITCH : " + pitchString);
        Serial.println("ROLL : " + rollString);
        
        message = "";
      }
    }
  }

  // Set current orientation and position
  inputYaw = ypr[0] * 180/M_PI;
  inputPitch = ypr[2] * 180/M_PI;
  inputRoll = ypr[2] * 180/M_PI;

  yawPID.Compute();
  pitchPID.Compute();
  rollPID.Compute();

  m1Throttle = throttle + pidOutRoll + pidOutPitch + pidOutYaw;
  m2Throttle = throttle - pidOutRoll + pidOutPitch - pidOutYaw;
  m3Throttle = throttle + pidOutRoll - pidOutPitch - pidOutYaw;
  m4Throttle = throttle - pidOutRoll - pidOutPitch + pidOutYaw;

  Serial.print("OutYaw : ");
  Serial.print((float)inputYaw,3);
  Serial.print("\t");
  Serial.print("OutPitch : ");
  Serial.print((float)inputPitch,3);
  Serial.print("\t");
  Serial.print("RollPID : ");
  Serial.print((float)inputRoll,3);
  Serial.print("\t");
  Serial.print((float)m1Throttle,3);
  Serial.print("\t");
  Serial.print((float)m2Throttle,3);
  Serial.print("\t");
  Serial.print((float)m3Throttle,3);
  Serial.print("\t");
  Serial.println((float)m4Throttle,3);
  
}
