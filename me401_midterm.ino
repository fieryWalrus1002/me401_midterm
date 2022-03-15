#include "main.h"
#include "ME401_Radio.h"
#include "ME401_PID.h"
//#include "dcmotor.h"
#include "contServo.h"
#include "ir_dist.h"
#include "navigation.h"

#define TESTSTATE AVOID
#define EXTINT 7 // interrupt 1 is on digital pin 2


/////////////////  PID TEST //////////////

// Global variables for the timer interrupt handling
//int pidSampleTime = 10;
//long counterPID=1;

// Global variables for the PID controller
double kp1=0.3,ki1=0.0,kd1=0.00; // stupid, simple, poor proportional controller


enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};
enum robotStates robotState = CAPTURE;
RobotPose myRobotPose;
navPoint currentTarget;

enum testStates {
  DISTANCE, // checks getDistance() values and prints to serial
  DISTSWEEP, // performs distanceSweep, gives best direction
  MOTORS, // performs a few movements
  AVOID // moves and avoids obstacles
};
enum testStates testState = TESTSTATE;


PID motorPid(&input, &output, &setpoint,kp1,ki1,kd1, DIRECT);           

void attack(){
  Serial.println("ATTACK");
}

void defend(){
  Serial.println("DEFEND"); 
}

void capture(){  
  currentTarget = home_base;
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
      avoidanceTest(); // tests how the robot avoids collisions while moving forward
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
  home_base.x = 1500.0;
  home_base.y = 1500.0;
    //Setup the pid 
             
  motorPid.SetMode(AUTOMATIC);
  motorPid.SetSampleTime(10);
  motorPid.SetOutputLimits(-1.0,1.0);
  // create external interrupt to keep robot from driving off the table while working on it
//  pinMode(EXTINT, INPUT);
//  attachInterrupt(digitalPinToInterrupt(EXTINT), onStateHandler, RISING);
  
  
  ME401_Radio_initialize(); // Initialize the RFM69HCW radio
  Serial.println("radio online");
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
  //  setupPIDandIR();
}

void loop() {
  checkStatus(); // check the status of the game environment
  //  changeState(); // check to see if a state change is called for
  handleState(); // execute current state function

  // update PID controller
  // calculate movement error
  double distError = calcDistError(currentTarget, myRobotPose);  //(navPoint navpoint, robotPose pose)
  input = myRobotPose.x;
  setpoint = currentTarget.x;
//  motorPid(&input, &output, &setpoint,kp,ki,kd, DIRECT);                       
//  PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
//  Serial.print("distError= ");
//  Serial.println(distError);  

  motorPid.Compute();
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.println(output);
  commandMotors(-1* output, -1* output);







}
