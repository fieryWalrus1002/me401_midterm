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
Servo leftServo;
Servo rightServo;

// function prototype
void commandMotors(double leftInput, double rightInput);

// Global variables for the motor PID controller
double kp1=0.3,ki1=0.0,kd1=0.00; // stupid, simple, poor proportional controller
double distError=0, velocity=0, distSetpoint=0;
PID velocityPid(&distError, &velocity, &distSetpoint,kp1,ki1,kd1, DIRECT);           

// Global variables for the steering PID controller
double kp2=0.3,ki2=0.0,kd2=0.00; // stupid, simple, poor proportional controller
double rotError=0, angleAdj=0, headingSetpoint=0;
PID headingPid(&rotError, &angleAdj, &headingSetpoint,kp2,ki2,kd2, DIRECT);           

void initMotors(){
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin); // higher makes it go backward
  leftServo.write(90);
  rightServo.write(90);
  
  // speed PID
  velocityPid.SetMode(AUTOMATIC);
  velocityPid.SetSampleTime(10);
  velocityPid.SetOutputLimits(-1.0,1.0);

  // rotation PID
  headingPid.SetMode(AUTOMATIC);
  headingPid.SetSampleTime(10);
  headingPid.SetOutputLimits(-1.0,1.0);
}

void updateMotors(navPoint pn_r){

  // calculate distance error (distance from setpoint to  nav point)
  distError = getDistanceRelRobot(pn_r);
  velocityPid.Compute();

  // calculate rotation error (difference between desired heading and measured
  rotError = getHeadingRelRobot(pn_r);
  headingPid.Compute();

  Serial.print(millis());
  Serial.print(", ");
  Serial.print(", ");
  Serial.print(distError);
  Serial.print(", ");
  Serial.println(velocity);
  Serial.print(", ");
  Serial.print(rotError);
  Serial.print(", ");
  Serial.println(angleAdj);
  commandMotors(velocity + angleAdj * 0.25, velocity - angleAdj * 0.25);
}

void commandMotors(double leftInput, double rightInput){
  /* take in a value of -1 to 1 for each motor, and turn that
  *  into the duty cycle to each motor. 
  */
  int leftServoDC;
  int rightServoDC;

  // map inputs to servo write values for pwm generation
  if (leftInput >= 0.0){
    leftServoDC = map(leftInput, 0.0, 1.0, 90, 180);
  } else if (leftInput < 0.0){
    leftServoDC = map(leftInput, -1.0, 0.0, 0, 90);
  }

  if (rightInput >= 0.0){
    rightServoDC = map(rightInput, 0.0, 1.0, 90, 0);
  } else if (rightInput < 0.0){
    rightServoDC = map(rightInput, -1.0, 0.0, 180, 90);
  }
  
  // write output duty cycle to servos
  leftServo.write(leftServoDC);
  rightServo.write(rightServoDC);
}

#endif
