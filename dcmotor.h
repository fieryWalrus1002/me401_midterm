/*  
 *  DC motor control
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-4-22
 *  For ME401 Spring 22
 *  
 *  chipKit Motor Driver
 *  JP1 and JP2 set to 5/34 for motor 2
 *  Power supplied to J1 from J14
 *  Reference Manual located at:
 *  https://digilent.com/reference/add-ons/motor-shield/reference-manual
 *  
 *  DIR1 EN1   Result
 *   0   0       Stop
 *   0   1/PWM   Fprward
 *   1   0       Stop
 *   1   1/PWM   Reverse
 *  DIR2 EN2     Result
 *   0   0       Stop
 *   0   1/PWM   Forward
 *   1   0       Stop
 *   1   1/PWM   Reverse 
*/

#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#define LEFTMOTOR_EN 3
#define LEFTMOTOR_DIR 4
#define RIGHTMOTOR_EN 5
#define RIGHTMOTOR_DIR 34
#define FORDIR 0
#define REVDIR 1
#define SLOW 50
#define FAST 150

enum moveActions {
  ALLSTOP,
  FORWARD,
  BACKUP,
  LEFTTURN,
  RIGHTTURN
};

int right_pwm = 0;
int left_pwm = 0;

void commandMotors(int leftDir, int leftPWM, int rightDir, int rightPWM){
  digitalWrite(LEFTMOTOR_DIR, leftDir);
  analogWrite(LEFTMOTOR_EN, leftPWM);
  digitalWrite(RIGHTMOTOR_DIR, rightDir);
  analogWrite(RIGHTMOTOR_EN, rightPWM);
}

void moveRobot(enum moveActions chosenAction, int delayTime){
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
    case LEFTTURN:
      Serial.println("LEFTTURN");
      commandMotors(REVDIR, SLOW, FORDIR, FAST);
      break;
    case RIGHTTURN:
      Serial.println("RIGHTTURN");
      commandMotors(FORDIR, FAST, REVDIR, SLOW);
      break;
    default:
      Serial.println("default, no move");
      break;
  }  

  delay(delayTime);
}


void initDcMotors(){
  pinMode(LEFTMOTOR_EN, OUTPUT);
  pinMode(LEFTMOTOR_DIR, OUTPUT);
  pinMode(RIGHTMOTOR_EN, OUTPUT);
  pinMode(RIGHTMOTOR_DIR, OUTPUT);

  moveRobot(ALLSTOP, 0);
  
}

void testMotors(){
  moveRobot(FORWARD, 250);
  moveRobot(LEFTTURN,100);
  moveRobot(FORWARD, 250);
  moveRobot(RIGHTTURN, 100);
  moveRobot(BACKUP, 250);
  moveRobot(FORWARD, 250);
  moveRobot(RIGHTTURN, 100);
  delay(2000);
}


#endif
