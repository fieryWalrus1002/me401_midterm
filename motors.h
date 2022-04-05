#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <Arduino.h>
#include "main.h"
#include "navsystem.h"
#include <Servo.h>

#define leftServoPin 32
#define rightServoPin 31
#define maxInput 1.0
#define minInput -1.0

struct RobotVars{
  float v;
  float l;
  float r;
  float tD;
  float tA;
  NavPoint pn_r;
  float currentDist;
  float desiredHeading;
  float angleAdj;
} robotVars;


struct PIDVars{
  // PID controller params
  double Kp;
  double Ki;
  double Kd;
  
  // stuff we need to keep track of
  double _integral;
  double _prevError;
  double _dT;
  double _currentError;
  double _setPoint;
  
  // max output limit for the controller
  double maxLimit;
  double minLimit;
};

class Motors {
    public:
        void update(NavPoint navpoint, bool debug);
        void commandMotors(double, double);
        double pidCalc(PIDVars*, double);
        void setHeadingKp(PIDVars*, int);
        void setVelocityKp(PIDVars*, int);
        PIDVars vVars = {1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, -100};
        PIDVars hVars = {1, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, -100};
        Servo leftServo;
        Servo rightServo;
        float distTh = 25;
        float rotTh = 0.01;
        double distError = 0.0;
        double velocity = 0.0;
        double rotError = 0.0;
        double angleAdj = 0.0;
        int leftServoDC;
        int rightServoDC;
        
        bool reverseMotors = false;

        

};

Motors motors;


#endif

