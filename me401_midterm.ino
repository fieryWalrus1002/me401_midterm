#include "main.h"
#include "ME401_Radio.h"
#include "ME401_PID.h"
//#include "dcmotor.h"
#include "contServo.h"
#include "ir_dist.h"
#include "navigation.h"

#define TESTSTATE AVOID
<<<<<<< HEAD
#define EXTINT 7 // interrupt 1 is on digital pin 2
=======
// comment to test changes
>>>>>>> 59944088d268e7bf9ce61e4b532413551bc6bfc4

enum robotStates {
  ATTACK, // search for balls in neutral and opposing base
  CAPTURE, // return with a ball to home base
  DEFEND, // interpose robot between opposing robots and home base
  TEST // for testing functionality
};
enum robotStates robotState = TEST;

enum testStates {
  DISTANCE, // checks getDistance() values and prints to serial
  DISTSWEEP, // performs distanceSweep, gives best direction
  MOTORS, // performs a few movements
  AVOID // moves and avoids obstacles
};
enum testStates testState = TESTSTATE;

volatile int robotOn = HIGH; // starts off, have to bring a pin low to start it

void attack(){
  Serial.println("ATTACK");
}

void defend(){
  Serial.println("DEFEND"); 
}

void capture(){
  Serial.println("CAPTURE");
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
      testMotors();
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
  printSelfPose();
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

void onStateHandler(){
  robotOn = !robotOn;
//  attachInterrupt(digitalPinToInterrupt(EXTINT), onStateHandler, HIGH);
  if (robotOn == LOW){
    motorStop();
  }
  
  Serial.println(robotOn);
}

void setup() {
  Serial.begin(115200);
  Serial.println("serial begin");
  
  initMotors(); // motors for locomotion
  Serial.println("dc motors online");
  initIrSensor(); // Sharp IR distance sensor initialize
  Serial.println("ir sensor online");

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
//  while(1){
    checkStatus(); // check the status of the game environment
    changeState(); // check to see if a state change is called for
    handleState(); // execute current state function

  
    // Here are a few examples of some of the core functionalities of the robot. If things ever stop working, I would
    // recommend keeping a copy of this original template around so that you can load it to your robot and check whether
    // you have a software issue or whether there is a hardware/wiring issue.
  
    
  
  //  RobotPose robot = getRobotPose(MY_ROBOT_ID);
  //  if (robot.valid == true)
  //  {
  //    Serial.println("The camera can see my robot");
  //    printRobotPose(robot);
  //  }
  //
  //  BallPosition ballPos[20];
  //  int numBalls = getBallPositions(ballPos);
  //  Serial.print("NUM BALLS: ");
  //  Serial.println(numBalls);
  //  printBallPositions(numBalls, ballPositions);
  //
  //// TODO: You will want to replace this serial output with something that
  //  // looks like:
  //  //    result of millis(),value in input,
  //  // on each line. Then you can copy this from the Serial Monitor into a 
  //  // text file, save it as a .csv file, then open it in Excel for plotting
  //  // to find the Ziegler-Nichols period.
  //  
  //  Serial.print("Setpoint: ");Serial.print(setpoint); Serial.print(" ");
  //  Serial.print("Measured : ");Serial.print(input); Serial.print(" ");
  //  Serial.print("PWM Output: ");Serial.print(output); Serial.print(" ");
  //  Serial.println("");
  //        
  

}
