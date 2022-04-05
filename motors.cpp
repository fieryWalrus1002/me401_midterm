/*  
 *  Motor PID and control functions
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 4-4-22
 *  For ME401 Spring 22
 *  
 *  Usage:
 * lets write that later
 *  
 *  
 *   
*/


#include "motors.h"
#include "navsystem.h"
void Motors::update(NavPoint navpoint, bool debug){
  static long prevTime = 0;
  long deltaT = millis() - prevTime;
  
  vVars._dT = deltaT;
  hVars._dT = deltaT;

  float targetDist = nav.getDistanceRelRobot(navpoint);
  float targetAngle = nav.getHeadingRelRobot(navpoint);

  robotVars.tD = targetDist;
  robotVars.tA = targetAngle;
  
  distError = (vVars._setPoint - targetDist) / 3111; // error normalized to max distance
  velocity = pidCalc(&vVars, distError); // calculate velocity from error with PID

  // calculate rotation error (difference between desired heading and measured
  rotError = (hVars._setPoint - targetAngle) / (2 * M_PI); // normalized to max angle value in radians
  angleAdj = pidCalc(&hVars, rotError);

   double leftMotorVal = (velocity + angleAdj);
   double rightMotorVal = (velocity - angleAdj);

    if (debug == true){
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
    }
     
   robotVars.l = leftMotorVal;
   robotVars.r = rightMotorVal;
   Motors::commandMotors(leftMotorVal, rightMotorVal);
   
}

void Motors::commandMotors(double leftInput, double rightInput){
    /* take in a value of -1 to 1 for each motor, and turn that into the duty cycle to 
    *  each motor. 
    *  
    *  The servos are oriented in opposite directions, so forward is 0 for right servo
    *  and forward is 180 for the left servo. 
    */
   
    int leftTemp = (leftInput * 90.0) + 90; // +1 is 180, -1 is 0. 
    int rightTemp = 90 - (rightInput * 90.0); // +1 is 0, -1 is 180

    leftServo.write(leftTemp);
    rightServo.write(rightTemp);
}

double Motors::pidCalc(PIDVars *vars, double currentError){
  vars->_integral = 0;
  double propError = (vars->Kp) * currentError;
  float dampError = (vars->Kd) * ((currentError - (vars->_prevError))); // derivative of last time step
  float intError = (vars->Ki) * vars->_integral;

  double output = propError + dampError + intError;
  vars->_prevError = currentError;
  
  if (output >(vars->maxLimit))
    output = vars->maxLimit;
  else if (output < (vars->minLimit))
    output = vars->minLimit;

  return output;
}

void Motors::setHeadingKp(PIDVars *vars, int newkp){
  double val = newkp / 100;
  vars->Kp = val;
}

void Motors::setVelocityKp(PIDVars *vars, int newkp){
  double val = newkp / 100;
  vars->Kp = val;
}
