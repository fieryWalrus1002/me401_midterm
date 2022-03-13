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
#include "motorPID.h"

#define leftServoPin 31
#define rightServoPin 32

Servo leftServo;
Servo rightServo;

void initMotors(){
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin); // higher makes it go backward
  
}




void commandMotors(int leftDir, int leftPWM, int rightDir, int rightPWM){
  int leftServoPWM = 0;
  int rightServoPWM = 0;
  
  if (leftDir > 0){
    leftServoPWM = map(leftPWM, 0, 255, STOPVAL, 0);
  } else {
    leftServoPWM = map(leftPWM, 0, 255, STOPVAL, 180);
  }

    if (rightDir > 0){
    rightServoPWM = map(rightPWM, 0, 255, STOPVAL, 180);
  } else {
    rightServoPWM = map(rightPWM, 0, 255, STOPVAL, 0);
  }
  leftServo.write(leftServoPWM);
  rightServo.write(rightServoPWM);
}





#endif
