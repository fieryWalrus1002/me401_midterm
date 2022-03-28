#ifndef _MAIN_H_
#define _MAIN_H_
#define MY_ROBOT_ID 2 // CHANGE THIS TO YOUR TEAM NUMBER
bool RADIO = true;
bool debugPID = false; // output PID paramters
bool debugOutput = false; // output position and angle


void attack();
void defend();
void capture();
void test();

#endif
