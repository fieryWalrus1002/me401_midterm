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
#define leftServoPin 32
#define rightServoPin 31
#define maxInput 1.0
#define minInput -1.0
Servo leftServo;
Servo rightServo;
int leftServoDC;
int rightServoDC;
unsigned long lastTime;
int sampleTime = 1000; // length of time between calls to update PID, in ms
const double distThresh = 5; // number of mm that we call "close enough"
const double angleThresh = 0.01; // arbitrary low "close enough" rotation threshold. helps control jitter at stop, due to changing coordinates on radio.
// function prototype
void commandMotors(double leftInput, double rightInput);

//PIDVars{Kp, Ki, Kd, _integral, _prevError, _dt, maxLimit, minLimit};
/////// heading PID controller
double rotError=0.0, angleAdj=0.0, headingSetpoint=0.0;
//PIDVars hVars = {0.04, 0.000049, 26.948, 0.0, 0.0, 0.0, 0.5, -0.5}; // calculated with Z-N method by Jon, but we modified max
PIDVars hVars = {0.04, 0, 0, 0.0, 0.0, 0.0, 1, -1};

////// velocity PID controller
double distError=0.0, velocity=0.0, distSetpoint=0.0;
PIDVars vVars = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1, -1};

/////// functions
void initMotors(){
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin); // higher makes it go backward
  leftServo.write(90);
  rightServo.write(90);
}

void updateMotors(navPoint navpoint, unsigned long deltaT, bool debug){
  // update deltaT in the vars for PID calculation
  vVars._dT = deltaT;
  hVars._dT = deltaT;

  // calculate distance error
  // normalize error to arbitrary maximum distance. It might be over 1, but not by too much. 
  // say, 800 away, so it would be 800/1000 = .8
  distError = (distSetpoint - getDistanceRelRobot(navpoint)) / 1200;
  velocity = pidCalc(&vVars, distError, false); 
   
  if (distError <= distThresh){ 
    velocity = 0;
  }

  // calculate rotation error (difference between desired heading and measured
  // normalized to pi, so 0 is dead on and 1 is almost directly away
  rotError = (headingSetpoint - getHeadingRelRobot(navpoint)) / M_PI;
  angleAdj = pidCalc(&hVars, rotError, true);
  
//  // account for "close enough" with angleThresh const
//  if (abs(rotError) <= angleThresh){
//      angleAdj = 0;
//      Serial.println("angle close enough");
//  }
  /* 
   * -angleAdj means the robot should turn left
   * so we will add it to the left wheel to reduce the left wheel velocity
   * and increase the right wheel velocity, so turning the robot left toward
   * the nav point
   * If the angleAdj is positive, it means the navPoint is toward the right. 
   * it will add to the left wheel, and reduce the right wheel velocity.
   * This will drive the robot toward the right. 
   */

   // we're zeroing out the velocity here to isolate the heading output's effects for tuning
//    velocity = 0;
   
   // When the velocity is high the values can become saturated. 
   Serial.print("vel: ");
   Serial.print(velocity);
   Serial.print(", angleAdj: ");
   Serial.println(angleAdj);
   double leftMotorVal = velocity + angleAdj;
   double rightMotorVal = velocity - angleAdj;
   double saturationVal = abs(velocity) + abs(angleAdj);

   if (leftMotorVal > 1 || leftMotorVal < -1 || rightMotorVal > 1 || rightMotorVal < -1){
      leftMotorVal /= saturationVal;
      rightMotorVal /= saturationVal;
      if (debugOutput == true){
        Serial.print("motorVals (SAT): ");
        Serial.print(leftMotorVal);
        Serial.print(", ");
        Serial.print(rightMotorVal);
        Serial.print(", saturation value: ");
        Serial.println(saturationVal);
      }
   } else {
      if (debugOutput == true){
        Serial.print("motorVals (UNSAT): ");
        Serial.print(leftMotorVal);
        Serial.print(", ");
        Serial.print(rightMotorVal);
        
      }
   }
   
   commandMotors(leftMotorVal, rightMotorVal);
}

void commandMotors(double leftInput, double rightInput){
  /* take in a value of -1 to 1 for each motor, and turn that
  *  into the duty cycle to each motor. 
  *  
  *  The servos are oriented in opposite directions, so forward is 0 for right servo
  *  and forward is 180 for the left servo. 
  */
  
  int leftTemp = (int)(leftInput * 90.0) + 90; // +1 is 180, -1 is 0. 
  int rightTemp = 90 - (int)(rightInput * 90.0); // +1 is 0, -1 is 180
  
  leftServo.write(leftTemp);
  rightServo.write(rightTemp);
}

#endif
