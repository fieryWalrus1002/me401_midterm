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
#define leftServoPin 31
#define rightServoPin 32
#define FORDIR 0
#define REVDIR 1
#define SLOW 225
#define FAST 150
#define STOPVAL 92
Servo leftServo;
Servo rightServo;

void initMotors(){
  leftServo.attach(leftServoPin);
  leftServo.write(STOPVAL); // higher makes it go forward
  rightServo.attach(rightServoPin); // higher makes it go backward
  rightServo.write(STOPVAL);
}

enum moveActions {
  ALLSTOP,
  FORWARD,
  BACKUP,
  TURNLEFT,
  TURNRIGHT
};

void motorStop(){
  leftServo.write(STOPVAL);
  rightServo.write(STOPVAL);
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

void moveRobot(enum moveActions chosenAction){
  //
  
  switch (chosenAction) {
    case ALLSTOP:
      Serial.println("STOP");
      commandMotors(FORDIR, 0, FORDIR, 0);
      break;
    case FORWARD:
      Serial.println("FORWARD");
      commandMotors(FORDIR, SLOW, FORDIR, SLOW);
      break;
    case BACKUP:
      Serial.println("BACKUP");
      commandMotors(REVDIR, SLOW, REVDIR, SLOW);
      break;
    case TURNLEFT:
      Serial.println("TURNLEFT");
      commandMotors(REVDIR, SLOW, FORDIR, SLOW);
      break;
    case TURNRIGHT:
      Serial.println("TURNRIGHT");
      commandMotors(FORDIR, SLOW, REVDIR, SLOW);
      break;
    default:
      Serial.println("default, no move");
      break;
  }  

}


//void initMotors(){
//  pinMode(LEFTMOTOR_EN, OUTPUT);
//  pinMode(LEFTMOTOR_DIR, OUTPUT);
//  pinMode(RIGHTMOTOR_EN, OUTPUT);
//  pinMode(RIGHTMOTOR_DIR, OUTPUT);
//
//  moveRobot(ALLSTOP);
//  
//}

void testMotors(){
  moveRobot(FORWARD);
  delay(2000);
  moveRobot(BACKUP);
  delay(2000);
  moveRobot(ALLSTOP);
  delay(10000);
}


#endif
