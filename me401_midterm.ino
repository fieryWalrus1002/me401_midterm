#include "main.h"
#include "ME401_Radio.h"
#include "ME401_PID.h"
//#include "dcmotor.h"

#include "ir_dist.h"
#include "navigation.h"
#include "contServo.h"

#define TESTSTATE AVOID
#define EXTINT 7 // interrupt 1 is on digital pin 2
bool RADIO = true;
long dT = 0.0;

/////////////////  PID TEST //////////////

// Global variables for the timer interrupt handling
//int pidSampleTime = 10;
//long counterPID=1;


enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};
enum robotStates robotState = CAPTURE;
RobotPose myRobotPose;
navPoint currentNavPoint;

enum testStates {
  DISTANCE, // checks getDistance() values and prints to serial
  DISTSWEEP, // performs distanceSweep, gives best direction
  MOTORS, // performs a few movements
  AVOID // moves and avoids obstacles
};
enum testStates testState = TESTSTATE;


//navPoint test_point;


void attack(){
  Serial.println("ATTACK");
}

void defend(){
  Serial.println("DEFEND"); 
}

void capture(){  
  currentNavPoint = home_base;
}

void test(){
//  Serial.println("TEST");
  switch (testState) {
    case DISTANCE:
      // prints out distance sensor value in cm
      Serial.println(getDistance(90));
      break;
    case DISTSWEEP:
      // distance sweep, picks clearest direction to go
      // returns radian heading
      Serial.println(distanceSweep());
      break;
    case MOTORS:
      // test motor functionality
//      testMotors();
      break;
    case AVOID:
      //avoidanceTest(); // tests? how the robot avoids collisions while moving forward
      break;
    default:
      Serial.println("no test state");
      break;
  }
}



/* print out the self pos in a csv style format for debug */
void printSelfPose(){
  RobotPose myPose = getRobotPose(MY_ROBOT_ID);
  if (myPose.valid == true)
  {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(MY_ROBOT_ID);
    Serial.print(", ");
    Serial.print(myPose.x);
    Serial.print(", ");
    Serial.print(myPose.y);
    Serial.print(", ");
    Serial.println(myPose.theta);
  }
}

void checkStatus(){
  updateRobotPoseAndBallPositions();
  myRobotPose = getRobotPose(MY_ROBOT_ID);
}

void handleState(){
//  Serial.println("handleState()");
   switch (robotState) {
    case 0:
      // Attack
      attack();
      break;
    case 1:
      // Capture, return to base with ball
      capture();
      break;
    case 2:
      // Defend
      defend();
      break;
    case 3:
      // test functionality
      test();
      break;
    default:
      // no state, switch to attack
      robotState = ATTACK;
      break;
  }
}

void changeState(){
//  Serial.println("changeState");
}


void setup() {
  Serial.begin(115200);
  Serial.println("serial begin");
  
  initMotors(); // motors for locomotion
  Serial.println("dc motors online");
  
  initIrSensor(); // Sharp IR distance sensor initialize
  Serial.println("ir sensor online");
  initNavSystem();
  
  if (RADIO == true){
    ME401_Radio_initialize(); // Initialize the RFM69HCW radio  
    Serial.println("radio online");
  }
  
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
  //  setupPIDandIR();
    currentNavPoint = {100,0};
    myRobotPose.x = 0;
    myRobotPose.y = 0;
    myRobotPose.theta = 0;
}

void loop() {
  dT = millis() - dT;
  
  if (RADIO == true){
    checkStatus(); // check the status of the game environment
    changeState(); // check to see if a state change is called for
    handleState(); // execute current state function
    
    // update the relative position of the nav point
    navPoint pn_r = getPnr(currentNavPoint, myRobotPose);
    
    // use data on relative distance to navPoint to update motor PIDs
    updateMotors(pn_r);
    
  }
}
