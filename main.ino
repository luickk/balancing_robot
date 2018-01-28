#include "PID_v1.h"
#include "Motor_Controller.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // true when DMP init was successfull
uint8_t mpuIntStatus; // interrupt status of gpu
uint8_t devStatus; // returns device status after every it.
uint16_t packetSize; // default is 42 bytes
uint16_t fifoCount; // count of all bytes currently in FIFO(FIFO is a cost flow assumption)
uint8_t fifoBuffer[64]; // FIFO(cost flow assumption) storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 185; // Setpoint robot is bound to
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1; // Defines angel offset in which robot is not counter moving
double input, output; // PID input value and output value

//PID values Proportional, Derivative, Integral term
double Kp = 40; 
/* 
Proportional term: this controls how quickly to turn the steering when the heading is not at the set value.

A low P will lead to sluggish steering, reacting only slowly to set heading changes. It may never reach the commanded value.
A higher P will give a snappier response, ideally with the steering turning rapidly and smoothly to follow commanded heading changes.
*/
double Kd = 1;
/*
Derivative term: this can be used to limit the speed of the steering response.

Assuming you already have a P term coarsely set in a sensible range, this will serve to reduce the speed of the steering response when the error is changing rapidly.
If your P term is set high and overshoot or oscillation is occurring, adding Derivative action will limit the amount of hard turning done for large steering commands.
In other words increasing the D term will reduce the maximum "sway" performed by the steering. Ideally this will serve to reduce overshoot so if the P controller is tuned too high, the right amount of Derivative action should stop oscillation.
Too much Derivative action will probably become sluggish in steering again.
*/
double Ki = 200;
/*
Integral term: this adds more steering action if the error persists for too long.

If the existing P+D control terms lead to an overall response that settles short of the required heading, an I term will slowly add a steering command to edge closer (we hope).
Too much Integral action can again cause overshoot/weaving, as the added Integral action turns in the correct way to get onto the commanded heading, but continues to steer because the large I term effectively has a long memory of steering in that direction... it will take a while for the integrated error to reduce and straighten up again.
*/

//best working:
//double Kp = 40;   
//double Kd = 1;
//double Ki = 200;

//Defining PID controller
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.4;
double motorSpeedFactorRight = 0.4;

//Motor controller pins
int ENA = 3;
int ENB = 9;
int IN1 = 4;
int IN2 = 5;
int IN3 = 11;
int IN4 = 10;

Motor_Controller motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
 Serial.begin(9600);
 Wire.begin();
 
 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-500, 500); 
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}


void loop()
{
 // if programming failed, don't try to do anything
 if (!dmpReady) return;

 // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //no mpu data - performing PID calculations and output to motors 
 pid.Compute();
 motorController.move(output, 20);
 }

 // reset interrupt flag and get INT_STATUS byte
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 // check for overflow (this should never happen unless our code is too inefficient)
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // reset so we can continue cleanly
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 // otherwise, check for DMP data ready interrupt (this should happen frequently)
 }
 else if (mpuIntStatus & 0x02)
 {
 // wait for correct available data length, should be a VERY short wait
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 // read a packet from FIFO
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 // track FIFO count here in case there is > 1 packet available
 // (this lets us immediately read more without waiting for an interrupt)
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;
 Serial.println(input);
 }
}
