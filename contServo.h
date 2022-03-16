/*  
 *  DC motor control
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-9-22
 *  For ME401 Spring 22
 *  
 *  chipKit Servo Motor Driver
 *  Pin# 
 *  26   A: PMD0/RE0   
 *  27   B: PMD1/RE1   
 *  28  C: PMD2/RE2   
 *  29  D: PMD3/RE3
 * 
 *  Power supplied to J1 from J14
 *  Reference Manual located at:
 *  https://digilent.com/reference/add-ons/motor-shield/reference-manual

*/
#ifndef _CONT_SERVO_H_
#define _CONT_SERVO_H_
#include <Servo.h>
//#include "motorPID.h"

#define leftServoPin 32
#define rightServoPin 31
#define maxInput 1.0
#define minInput -1.0
#define LOFFSET 2 // offset of servo value to get no motion. 92 in this case
#define ROFFSET 2.5 //
Servo leftServo;
Servo rightServo;

// function prototype
void commandMotors(double leftInput, double rightInput);

// Global variables for the motor PID controller
double kp1=0.3,ki1=0.0,kd1=0.00; // stupid, simple, poor proportional controller
double distErr=0, velocity=0, targetDist=0;
PID motorPid(&distErr, &velocity, &targetDist,kp1,ki1,kd1, DIRECT);           

// Global variables for the steering PID controller
double kp2=0.3,ki2=0.0,kd2=0.00; // stupid, simple, poor proportional controller
double rotErr=0, steer=0, heading=0;
PID steerPid(&rotErr, &steer, &heading,kp2,ki2,kd2, DIRECT);           

void initMotors(){
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin); // higher makes it go backward

  // speed PID
  motorPid.SetMode(AUTOMATIC);
  motorPid.SetSampleTime(10);
  motorPid.SetOutputLimits(-1.0,1.0);

  // rotation PID
  steerPid.SetMode(AUTOMATIC);
  steerPid.SetSampleTime(10);
  steerPid.SetOutputLimits(-1.0,1.0);

}


void updateMotors(navPoint navpoint, RobotPose pose){
  // calculate movement error
  distError = calcDistError(navpoint, pose);  //(navPoint navpoint, robotPose pose)
  targetDist = currentTarget.x;
  motorPid.Compute();

  // calculate roation error
  rotError = calcRotError(navpoint, pose);
  steerPid.Compute();

  // calculate steering error
  double rotErr=0, steer=0, heading=0;

  Serial.print(millis());
  Serial.print(", ");
  Serial.print(distErr);
  Serial.print(", ");
  Serial.print(targetDist);
  Serial.print(", ");
  Serial.println(velocity);
  Serial.print(", ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(rotErr);
  Serial.print(", ");
  Serial.println(steer);
  commandMotors(output + steer, output - steer);
}

void commandMotors(double leftInput, double rightInput){
  /* take in a value of -1 to 1 for each motor, and turn that
  *  into the duty cycle to each motor. 
  */
  int leftServoDC;
  int rightServoDC;

  // map inputs to servo write values for pwm generation
  if (leftInput >= 0.0){
    leftServoDC = map(leftInput, 0.0, 1.0, 90 + LOFFSET, 180);
  } else if (leftInput < 0.0){
    leftServoDC = map(leftInput, -1.0, 0.0, 0, 90 - LOFFSET);
  }

  if (rightInput >= 0.0){
    rightServoDC = map(rightInput, 0.0, 1.0, 90 + ROFFSET, 0);
  } else if (rightInput < 0.0){
    rightServoDC = map(rightInput, -1.0, 0.0, 180, 90 + ROFFSET);
  }
  
  // write output duty cycle to servos
  leftServo.write(leftServoDC);
  rightServo.write(rightServoDC);
}

#endif
