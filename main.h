#ifndef _MAIN_H_
#define _MAIN_H_
#define MY_ROBOT_ID 2 // CHANGE THIS TO YOUR TEAM NUMBER
#include <SoftwareSerial.h>

bool RADIO = true;
bool debugPID = false; // output PID paramters
bool debugOutput = false; // output position and angle



SoftwareSerial BTSerial(34, 35);

struct PIDVars{
  // PID controller params
  double Kp;
  double Ki;
  double Kd;
  
  // stuff we need to keep track of
  double _integral;
  double _prevError;
  double _dT;
  
  // max output limit for the controller
  double maxLimit;
  double minLimit;
};

void attack();
void defend();
void capture();
void test();




#endif
