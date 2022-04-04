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
#include "main.h"
#include "btserial.h"
#include "navigation.h"

#define leftServoPin 32
#define rightServoPin 31
#define maxInput 1.0
#define minInput -1.0
Servo leftServo;
Servo rightServo;
int leftServoDC;
int rightServoDC;
unsigned long lastTime;
bool reverseMotors = false;
float distTh = 25;
float rotTh = 0.01;
struct RobotVars{
  float v;
  float l;
  float r;
  float tD;
  float tA;
  navPoint pn_r;
  float currentDist;
  float desiredHeading;
  float angleAdj;
} robotVars;

//  navPoint pn_r = getPnr(currentNavPoint, myRobotPose);
//  float currentDist = getDistanceRelRobot(pn_r);
//  float desiredHeading = getHeadingRelRobot(pn_r);
//  float x = currentNavPoint.x;
//  float y = currentNavPoint.y;

float const ERRDECAY = 0.1; // rate of integral decay

int sampleTime = 1000; // length of time between calls to update PID, in ms
const double distThresh = 5; // number of mm that we call "close enough"
const double angleThresh = 0.001; // arbitrary low "close enough" rotation threshold. helps control jitter at stop, due to changing coordinates on radio.
// function prototype
void commandMotors(double leftInput, double rightInput);
double pidCalc(PIDVars *vars, double currentError, bool debug);

//PIDVars{Kp, Ki, Kd, _integral, _prevError, _dt, maxLimit, minLimit};
/////// heading PID controller
double rotError=0.0, angleAdj=0.0, headingSetpoint=0.0;

//PIDVars hVars = {0.5, 0.000049, 26.948, 0.0, 0.0, 0.0, 0.5, -0.5}; // calculated with Z-N method by Jon, but we modified max//

PIDVars hVars = {1, 0, 0, 0.0, 0.0, 0.0, 100, -100};

////// velocity PID controller
double distError=0.0, velocity=0.0, distSetpoint=0.0;
PIDVars vVars = {1, 0.0, 0.0, 0.0, 0.0, 0.0, 100, -100};

/////// functions
void initMotors(){
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin); // higher makes it go backward
  leftServo.write(90);
  rightServo.write(90);
}

void updateMotors(navPoint navpoint, unsigned long deltaT, bool debug){
  static long prevTime = 0;
  long now = millis();
  
  // update deltaT in the vars for PID calculation
  vVars._dT = deltaT;
  hVars._dT = deltaT;

  float targetDist = getDistanceRelRobot(navpoint);
  float targetAngle = getHeadingRelRobot(navpoint);

  robotVars.tD = targetDist;
  robotVars.tA = targetAngle;
  
  // calculate distance error
  distError = (distSetpoint - targetDist) / 3111; // normalized to max distance
  velocity = pidCalc(&vVars, distError, false); 
   
//  velocity = 0;

  // calculate rotation error (difference between desired heading and measured
  rotError = (headingSetpoint - targetAngle) / M_PI; // normalized to max angle value in radians
  angleAdj = pidCalc(&hVars, rotError, true);

  // check to see how close you are
  if (targetDist < distTh){
    BTSerial.println("we're here!");
    angleAdj = 0;
    velocity = 0;
    setWaypointFlag(true);
  }



   double leftMotorVal = (velocity + angleAdj);
   double rightMotorVal = (velocity - angleAdj);

  Serial.print(millis());
  Serial.print(", v: ");
  Serial.print(targetDist);
  Serial.print(", ");
  Serial.print(distError);
  Serial.print(", ");
  Serial.print(velocity);   
   Serial.print(", h: ");
  Serial.print(targetAngle);
  Serial.print(", ");
  Serial.print(rotError);
  Serial.print(", ");
  Serial.println(angleAdj);   
   Serial.print(", movtor Val: (");
   Serial.print(leftMotorVal);
   Serial.print(", ");
   Serial.print(rightMotorVal);
   Serial.println(")");
   
   robotVars.l = leftMotorVal;
   robotVars.r = rightMotorVal;
   commandMotors(leftMotorVal, rightMotorVal);

}

void commandMotors(double leftInput, double rightInput){
  /* take in a value of -100 to 100 for each motor, and turn that
  *  into the duty cycle to each motor. 
  *  
  *  The servos are oriented in opposite directions, so forward is 0 for right servo
  *  and forward is 180 for the left servo. 
  */
   
   int leftTemp = (leftInput * 90.0) + 90; // +1 is 180, -1 is 0. 
    int rightTemp = 90 - (rightInput * 90.0); // +1 is 0, -1 is 180

  leftServo.write(leftTemp);
  rightServo.write(rightTemp);
  Serial.print(", (");
  Serial.print(leftTemp);
  Serial.print(", ");
  Serial.print(rightTemp);
  Serial.println(")");
  
}



double pidCalc(PIDVars *vars, double currentError, bool debug){
  //  vars->_integral += vars->_integral * (1 - ERRDECAY) + currentError; // integral error weighted towards current error, old error values less weight

  double propError = (vars->Kp) * currentError;
  float dampError = (vars->Kd) * ((currentError - (vars->_prevError))); // derivative of last time step
  //  float intError = (vars->Ki) * vars->_integral;

  double output = propError + dampError;
//  double output = propError;  
  vars->_prevError = currentError;
  
  if (output >(vars->maxLimit))
    output = vars->maxLimit;
  else if (output < (vars->minLimit))
    output = vars->minLimit;

  return output;
}

void setHeadingKp(PIDVars *vars, int newkp){
  double val = newkp / 100;
  vars->Kp = val;
}

void setVelocityKp(PIDVars *vars, int newkp){
  double val = newkp / 100;
  vars->Kp = val;
}

#endif
