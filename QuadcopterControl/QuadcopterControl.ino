#include "MPU6050.h"
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
#define MAX_PID      100
#define MIN_PID     -100

#define LOOP_PERIOD  0.03
#define GYRO_SCALE   131
#define RAD_TO_DEG   57.29578
#define M_PI         3.14159265358979323846

// ESC Variables -------------------------------------------------------- |
Servo m1;
Servo m2;
Servo m3;
Servo m4;

// IMU / MPU Variables -------------------------------------------------- |
MPU6050 mpu;

float startTime;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float gxAngle=0, gyAngle=0, gzAngle=0;
float axAngle=0, ayAngle=0, azAngle=0;
float yaw = 0, pitch = 0, roll = 0;

// PID ----------------------------------------------------------------- |
float setYaw      = 0;
float setPitch    = 0;
float setRoll     = 0;
float inputYaw    = 0;
float inputPitch  = 0;
float inputRoll   = 0;
float pidOutYaw   = 0;
float pidOutPitch = 0;
float pidOutRoll  = 0;

float yawOffset   = 0;
float pitchOffset = 0;
float rollOffset  = 0;

// tuning parameters
float Kp = 5;
float Ki = 5;
float Kd = 1;

PID yawPID  (&inputYaw,   &pidOutYaw,   &setYaw,   Kp, Ki, Kd, DIRECT);
PID pitchPID(&inputPitch, &pidOutPitch, &setPitch, Kp, Ki, Kd, DIRECT);
PID rollPID (&inputRoll,  &pidOutRoll,  &setRoll,  Kp, Ki, Kd, DIRECT);

// Throttle / Parsing Variables ---------------------------------------- |
float throttleIn = 1000;
float throttle = 1000;
float m1Throttle;
float m2Throttle;
float m3Throttle;
float m4Throttle;

bool readThrottle = false;
bool readPos = false;

String message = "";

// Setup =================================================================================|
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

// Loop ==================================================================================|
void loop()
{
    startTime = millis();
    
    // Receive MPU data ------------------------------------------------------------------|
    // Read raw accel/gyro measurements from device; IMU is sideways, so swap x and z
    mpu.getMotion9(&az, &ay, &ax, &gz, &gy, &gx, &mz, &my, &mx);

    // gyro sensitivity scale factor for 250 dps : 0.131
    gx = (float) gx / GYRO_SCALE;
    gy = (float) gy / GYRO_SCALE;
    gz = (float) gz / GYRO_SCALE;
    
    // invert x due to orientation
    gx = -gx;
    
    // calculate gyro angles using angular rotation rate and period between samples
    gxAngle += gx * LOOP_PERIOD;
    gyAngle += gy * LOOP_PERIOD;
    gzAngle += gz * LOOP_PERIOD;

    // convert accelerometer samples to angles in degrees
    axAngle = (float)(atan2(ay,az) + M_PI) * RAD_TO_DEG;
    ayAngle = (float)(atan2(ax,az) + M_PI) * RAD_TO_DEG;
    azAngle = (float)(atan2(ax,ay) + M_PI) * RAD_TO_DEG;

    //change rotation val of accelerometer to +/- 180
    if (axAngle > 180) axAngle -= 360;
    if (ayAngle > 180) ayAngle -= 360;
    if (azAngle > 180) azAngle -= 360;

    // filter and combine gyro and accel angles
    yaw   = 0.98 * (yaw   + gz * LOOP_PERIOD) + (1 - 0.98) * azAngle;
    pitch = 0.98 * (pitch + gx * LOOP_PERIOD) + (1 - 0.98) * axAngle;
    roll  = 0.98 * (roll  + gy * LOOP_PERIOD) + (1 - 0.98) * ayAngle;

    //Serial.print(yaw); Serial.print("\t");
    //Serial.print(pitch); Serial.print("\t");
    //Serial.print(roll); Serial.print("\t\t\t");

    // Receive user control data via bluetooth -------------------------------------------|
    //Read from bluetooth and write to usb serial
    if(Serial.available())
    {
        char charIn = (char)Serial.read();
        
        // Throttle value detected
        if (charIn == 't') {
          Serial.println("READING THROTTLE VALUE");
          readThrottle = true;
          readPos = false;

        // Position value detected
        } else if (charIn == 'p') {
          Serial.println("READING POSITION VALUE");
          readThrottle = false;
          readPos = true;

        // Read in data
        } else {
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
      
            // Reading in position values
            } else if (readPos) {
                parseYawPitchRoll(message, inputYaw, inputPitch, inputRoll);
                message = "";
            }
        }
    }

    // Set current orientation and position
    //inputYaw   = ypr[0] * 180/M_PI + yawOffset;
    inputPitch  = pitch * 180/M_PI + pitchOffset;
    inputRoll = roll * 180/M_PI + rollOffset;
    
  
    yawPID.Compute();
    pitchPID.Compute();
    rollPID.Compute();
  
    m1Throttle = throttle + pidOutRoll - pidOutPitch; // + pidOutYaw;
    m2Throttle = throttle - pidOutRoll - pidOutPitch; // - pidOutYaw;
    m3Throttle = throttle + pidOutRoll + pidOutPitch; // - pidOutYaw;
    m4Throttle = throttle - pidOutRoll + pidOutPitch; // + pidOutYaw;
  
    //Serial.print("OutYaw : ");
    //Serial.print((float)inputYaw,3);
    //Serial.print("\t");
    //Serial.print("OutPitch : ");
    //Serial.print((float)inputPitch,3);
    //Serial.print("\t");
    //Serial.print("RollPID : ");
    //Serial.print((float)inputRoll,3);
    //Serial.print("\t");
    //Serial.print((float)m1Throttle,3);
    //Serial.print("\t");
    //Serial.print((float)m2Throttle,3);
    //Serial.print("\t");
    //Serial.print((float)m3Throttle,3);
    //Serial.print("\t");
    //Serial.print((float)m4Throttle,3);

    float    period = millis() - startTime;
    Serial.print("Total time : "); Serial.print(period);
    
    // ensure each loop is at least 30ms
    while(millis() - startTime < 30){
      delayMicroseconds(100);
    }
    period = millis() - startTime;
    Serial.print("Total time after: "); Serial.println(period);
  
}

void parseYawPitchRoll (String data, float &yaw, float &pitch, float &roll) {
    int indexCommaA = message.indexOf(',');
    int indexCommaB = message.indexOf(',', indexCommaA + 1);

    String yawString   = message.substring(0, indexCommaA);
    String pitchString = message.substring(indexCommaA + 1, indexCommaB);
    String rollString  = message.substring(indexCommaB + 1);

    yaw   = atof(yawString.c_str());
    pitch = atof(pitchString.c_str());
    roll  = atof(rollString.c_str());
}

