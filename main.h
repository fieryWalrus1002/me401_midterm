/*  
 *  Main header file
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-24-22
 *  For ME401 Spring 22
 *  
 *
 *  
 *  
 *   
*/


#ifndef _MAIN_H_
#define _MAIN_H_
#define MY_ROBOT_ID 2 // CHANGE THIS TO YOUR TEAM NUMBER
#include <SoftwareSerial.h>
#include "navsystem.h"
#include "motors.h"


enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};

enum robotStates robotState = ATTACK;
SoftwareSerial BTSerial(34, 35);
RobotPose myRobotPose = {true, MY_ROBOT_ID, 500, 500, 0};
NavPoint pn_r;
bool RADIO = true;
bool serialDebug = true;

void attack();
void defend();
void capture();

#endif
