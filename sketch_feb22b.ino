#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <Stepper.h>
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 4, 3, 6, 7);
int stepCount = 0; 
//
//#define leftMotorPWMPin   3
//#define leftMotorDirPin   4
//#define rightMotorPWMPin  6
//#define rightMotorDirPin  7

#define Kp  0.1
#define Kd  0.1
#define Ki  100
#define sampleTime  0.005
#define targetAngle 91.40

MPU6050 mpu;

int16_t gyroY, accX, accZ;
volatile int gyroRate;
volatile float accAngle, gyroAngle=0, gyroangle=0, currentAngle, prevAngle=0,error,errorSum=0,motorPower;
unsigned long currTime, prevTime=0, loopTime;
volatile byte count=0;


void setMotors(float leftMotorSpeed, float rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255+leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin,  255+rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

void setup() {
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT)
  mpu.initialize();
  Serial.begin(9600);

}

void loop() {
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();
  gyroY = mpu.getRotationY();


  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroangle + (float)gyroRate*loopTime/1000;
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  error = currentAngle - targetAngle;
  errorSum = errorSum + error; 
  errorSum = constrain(errorSum, -30, 20);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  motorPower = constrain(motorPower, 0, 400);

  Serial.println(motorPower);
  myStepper.setSpeed(200);
  prevAngle = currentAngle;
}
