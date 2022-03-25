#ifndef _MAIN_H_
#define _MAIN_H_
#define MY_ROBOT_ID 2 // CHANGE THIS TO YOUR TEAM NUMBER
bool RADIO = true;
bool debugPID = true; // output PID paramters
bool debugOutput = true; // output position and angle

void attack();
void defend();
void capture();
void test();
//navPoint currentNavPoint = {2000.0, 500.0};
#endif
