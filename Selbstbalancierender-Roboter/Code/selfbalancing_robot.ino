
// PID-code based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

AccelStepper leftWheel(1, 2, 5); // define stepper for left wheel
AccelStepper rightWheel(1, 4, 7); // define stepper for right wheel
int maxSpeed = 3000; // in steps per second

MPU6050 mpu(0x68); // I2C adress by default is 0x68

bool dmpReady = false;
uint8_t devStatus;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
int lastMPUtime = 0;

Quaternion q; // [w,x,y,z]
VectorFloat gravity; // [x,y,z]
float ypr[3]; // [yaw,pitch,roll]

// PID Controller
double Kp=200.0, Ki=15, Kd=50; // PID-controller gain
double setPoint = 0, motSpeed = 0.0, robotTiltAngle = 0.0; // setpoint, output, input
double prevError=0.0, sumError=0.0;
unsigned long prevTime=0;
int sampleTime = 1; // in ms

// #########################################
// #            SETUP                      #
// #########################################

void setup() {

  leftWheel.setMaxSpeed(maxSpeed);
  rightWheel.setMaxSpeed(maxSpeed);

  // join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire:setup(400, true);
  #endif

  // initialize serial communication
  Serial.begin(115200);
  
  // initialize mpu
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-215);
  mpu.setYGyroOffset(108);
  mpu.setZGyroOffset(44);
  mpu.setZAccelOffset(628);

  if (devStatus == 0){
    mpuSetupRoutine();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  setPIDgains(Kp, Ki, Kd);

}

// #########################################
// #        MAINLOOP                       #
// #########################################

void loop() {
  if (dmpReady){
    if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      robotTiltAngle = ypr[2] * 180 / M_PI; // convert to degree
      computePID(); // compute PID-Control
      // Serial.println(robotTiltAngle * 180/M_PI);
    }
  }

  leftWheel.setSpeed(motSpeed);
  rightWheel.setSpeed(motSpeed);
  leftWheel.runSpeed();
  rightWheel.runSpeed();
}

// #########################################
// #        FUNCTIONS                      #
// #########################################

void computePID(){

  unsigned long curTime = millis();
  double dTime = (double)(curTime - prevTime);

  if (dTime >= sampleTime)
  {
    double error = setPoint - robotTiltAngle; // current error
    sumError += error; // integral of error
    double dError = (error - prevError); // derivation of error

    double Iterm = sumError * Ki;
    double Dterm = dError * Kd;
    double Pterm = error * Kp;

    // Anti wind up
    if (Iterm > maxSpeed) Iterm = maxSpeed;
    else if(Iterm < (-maxSpeed)) Iterm = -maxSpeed;
  
    motSpeed = Pterm + Iterm + Dterm; // output

    // Anti wind up
    if (motSpeed > maxSpeed) motSpeed = maxSpeed;
    else if(motSpeed < (-maxSpeed)) motSpeed = -maxSpeed;

    prevTime = curTime;
    prevError = error;
  }
}

void setSampleTime(int newSampleTime){

  if (newSampleTime > 0){
    double ratio = (double)newSampleTime / (double)sampleTime;
    Ki *= ratio;
    Kd /= ratio;
    sampleTime = (unsigned long)newSampleTime;
  }
}

void setPIDgains(double Kp, double Ki, double Kd){
  double sampleTimeInSec = ((double)sampleTime)/1000;
  Kp = Kp;
  Ki = Ki * sampleTimeInSec;
  Kd = Kd / sampleTimeInSec;
}

void mpuSetupRoutine(){

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

}