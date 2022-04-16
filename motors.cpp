    /*  
    *  Motors class and functions for controlling the robot movement in space
    *  Written by Magnus Wood (magnus.wood@wsu.edu) on 4-4-22
    *  Modified 4-6-22
    *  For ME401 Spring 22 miderm robot project
    * 
    * 
    * Structs:
    *   - RobotVars: I was thinking of usign this as a container that could be updated
    *       by functions whenever variables changed, so I could send a single struct
    *       by serial and get a great deal of information out instead of printing out
    *       csv type values. I ahven't finished it yet. 
    * 
    *   - PIDVars: a struct containing useful information about the PID variables, used
    *       to keep track of integral, limits, k gains, etc. Each PID controller needs
    *       its own vars. hVars is heading, vVars is velocity PID controlle vars.
    * 
    * Classes:
    *   - motors, Motors(): the class and the object of the motors. Keeps motor
    *       functions organized.
    *       Functions:
    *           - setHeadingKp(): convenience function for setting Kp for heading
    *               via serial debug. Intended to be used to control the tightness of
    *               turns by dynamically modifying Kp of heading PID controller.
    *               
    *           - setVelocityKp(): same, but for velocity. 
    *           - commandMotors(double, double): Take a -1 to +1 value and turn that
    *               into the 0 - 180 values we feed to the Servo class for each motor.
    *           - pidCalc(PIDVars*, double): returns a double corresponding to PID
    *               output. Does all the calculations, using the variables passed in
    *               as PIDVars, and the input being the error. 
    * 
    *           - update(NavPoint navpoint, bool debug):  This function does too much.
    *               It is the function responsible for keeping the robot moving toward
    *               its goal. It should be refactored. 
    *               Currently it takes in the goal navpoint and then:
    *                   1.  uses navsystem functions to calculate the position, 
    *                   heading and distance of the navpoint relative to the robot
    *                   2. calculates and normalizes error to 0-1 range
    *                   3. feeds that error to the piDCalc() functions
    *                   4. takes output and passes it to commandMotors()
    *                 
    *               
    * 
    */

#include "motors.h"
#include "navsystem.h"

void Motors::update(NavPoint navpoint, bool reachedPoint){
  static long prevTime = 0;
  long deltaT = millis() - prevTime;

  if (reachedPoint == true){
    commandMotors(0, 0);
    return;
  }
  
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

   if (headingTune == true){
    static long lastTime = 0;
    long thisTime = millis();
    if (thisTime - lastTime > 200){
      velocity = 0;
      Serial.print(millis());
      Serial.print(", ");
      Serial.print(targetAngle);
      Serial.print(", ");
      Serial.println(rotError);
      lastTime = lastTime;
    }
    
   }
   double leftMotorVal = (velocity + angleAdj);
   double rightMotorVal = (velocity - angleAdj);
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
   
    double leftTemp = (leftInput * 90.0) + 90; // +1 is 180, -1 is 0. 
    double rightTemp = 90 - (rightInput * 90.0); // +1 is 0, -1 is 180

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

void openGate(bool gateStatus){
  if (gateStatus == true){
    gateServo.write(GATE_OPEN_ANGLE);
  } else {
    gateServo.write(GATE_CLOSED_ANGLE);
  }
}

void orientSunbeam(){
  // find relative angle to closest robot. closestRobot navpoint is relative to robot
  double rads = nav.getHeadingRelRobot(nav.closestRobotXY);
  double degree = nav.convRadDegs(rads) + 90; // -90 would be 0, and 90 would be 180
  if (degree > 0 && degree < 180){
    sunbeamServo.write(degree);
  } else {
    sunbeamServo.write(90);
  }
}
  
 
