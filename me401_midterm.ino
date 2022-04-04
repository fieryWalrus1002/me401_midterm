#include "main.h"
#include "ME401_Radio.h"
#include "ME401_PID.h"

#include "btserial.h"
//#include "dcmotor.h"
#include "ir_dist.h"
#include "navigation.h"
#include "contServo.h"
#include "navlist.h"

#define TESTSTATE AVOID
#define EXTINT 7 // interrupt 1 is on digital pin 2


/////////////////  PID TEST //////////////

// Global variables for the timer interrupt handling
//int pidSampleTime = 10;
//long counterPID=1;
unsigned long lastdT = 0;
unsigned long dT = 0;

RobotPose myRobotPose = {true, 2, 500, 500, 0};


enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};
enum robotStates robotState = ATTACK;

enum testStates {
  DISTANCE, // checks getDistance() values and prints to serial
  DISTSWEEP, // performs distanceSweep, gives best direction
  MOTORS, // performs a few movements
  AVOID // moves and avoids obstacles
};
enum testStates testState = TESTSTATE;


void attack(){
//  Serial.println("ATTACK");
}

void defend(){
//  Serial.println("DEFEND"); 
}

void capture(){  
//  currentNavPoint = home_base;
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
    Serial.print(millis() - lastTime);
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
  waypointReached = checkWaypointStatus();
  if (waypointReached == true){
    currentNavPoint = navlist.getNextNavPoint(); // 
    setWaypointFlag(false);
  }
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

void pidSerialOutput(navPoint pn_r){
  /*  Serial output the pid values for Ziegler-Nichols tuning of the
   *  pid controller. 
   */
  
  Serial.print(millis());
  Serial.print(", ");
  Serial.print("(");
  Serial.print(pn_r.x);
  Serial.print(" ");
  Serial.print(pn_r.y);
  Serial.print("), ");
  Serial.print(rotError);
  Serial.print(", ");
  Serial.println(angleAdj);
}

void setup() {
  Serial.begin(115200);
  Serial.println("serial begin");
  
  initMotors(); // motors for locomotion
  Serial.println("dc motors online");
  
  initIrSensor(); // Sharp IR distance sensor initialize
  Serial.println("ir sensor online");


  if (RADIO == true){
    ME401_Radio_initialize(); // Initialize the RFM69HCW radio  
    checkStatus();
    Serial.println("radio online");
  } else {
    Serial.println("offline mode");
  }

  initNavSystem(&myRobotPose); // requires radio and checkStatus() run already
  Serial.println("nav system online");

  initBtSerial();
  Serial.println("btserial online");
  attachCoreTimerService(btDebugCallback);
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
  //  setupPIDandIR();
}

void loop() {
  // check the BTSerial for instructions
  while (BTSerial.available())
  {
      process_inc_byte(BTSerial.read());
  }


    
// 
//
  // set the time in seconds
  dT = (millis() - lastTime) / 1000;
 
  // update the environment and change states as required
   if (RADIO == true){
      checkStatus(); // check the status of the game environment
      changeState(); // check to see if a state change is called for
      handleState(); // execute current state function
   } else {
//    myRobotPose = {true, 2, 500, 500, 0};
//    currentNavPoint = {1000, 1000};
  }
 Serial.print("np: (");
 Serial.print(currentNavPoint.x);
  Serial.print(", ");
  Serial.print(currentNavPoint.y);
  Serial.print(")");


  // update the relative position of the nav point
  navPoint pn_r = getPnr(currentNavPoint, myRobotPose);

  // use navpoint and robot pose to calculate PID changes needed and modify motor output
  updateMotors(pn_r, dT, debugPID);

  lastdT = dT;
}

uint32_t btDebugCallback(uint32_t currentTime) {
  navPoint pn_r = getPnr(currentNavPoint, myRobotPose);
  float currentDist = getDistanceRelRobot(pn_r);
  float desiredHeading = getHeadingRelRobot(pn_r);
  float x = currentNavPoint.x;
  float y = currentNavPoint.y;
  String outputBuf = (String)millis() + "," + (String)x + "," + (String)y + "," + (String)currentDist + "," + (String)velocity  + "," + (String)desiredHeading  + "," + (String)angleAdj + ";";
  BTSerial.println(outputBuf);

  return (currentTime + CORE_TICK_RATE * 2000);
}
