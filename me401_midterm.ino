#include "main.h"
//#include "ME401_Radio.h"
#include "ME401_PID.h"
#include "btserial.h"
#include "ir_dist.h"
#include "navsystem.h"
#include "newRadio.h"

#define TESTSTATE AVOID
#define EXTINT 7 // interrupt 1 is on digital pin 2

void attack(){
  // 
}

void defend(){
//  Serial.println("DEFEND"); 
  goalPoint = home_base;
}

void capture(){  
//  currentNavPoint = home_base;
}

void checkStatus(){
    
    // update all robot, ball and global obstacle positions with radio data
    comms.updateRobotPoseAndBallPositions();

    // assign our robot's information to a robotPose so we can use it easily
    myRobotPose = comms.getRobotPose(MY_ROBOT_ID);

    // update the navigation systems data
    nav.update(&currentNavPoint);
   
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
      // test functionality, not defined at this time
      break;
    default:
      // no state, switch to attack
      robotState = ATTACK;
      break;
  }
}

void changeState(){
//  Serial.println("changeState");
//    if (robotState == ATTACK){
//      if (numBalls == 0){
//        robotState = DEFEND;
//      }
//    }
}



void setup() {
  Serial.begin(115200);
  Serial.println("serial begin");
    
  initIrSensor(); // Sharp IR distance sensor initialize
  Serial.println("ir sensor online");

  if (RADIO == true){
    comms.ME401_Radio_initialize(); // Initialize the RFM69HCW radio  
    checkStatus();
    Serial.println("radio online");
  } else {
    Serial.println("offline mode");
  }

  initBtSerial();
  Serial.println("btserial online");

  motors.leftServo.attach(leftServoPin);
  motors.rightServo.attach(rightServoPin);

  nav.setHomeBase(robotPoses[MY_ROBOT_ID]);
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
  //  setupPIDandIR();
  
  // serial output every 2s
  attachCoreTimerService(btDebugCallback);

  // update the goal position every 500 ms
  attachCoreTimerService(updateCallback);
}

void loop() {
  // check the BTSerial for instructions
  while (BTSerial.available())
  {
      process_inc_byte(BTSerial.read());
  }
 
  // update the environment and change states as required
   if (RADIO == true){
      checkStatus(); // check the status of the game environment
      changeState(); // check to see if a state change is called for
      handleState(); // execute current state function
   }

  // check to see if we have a clear path to our goal point
  nav.checkPath(&currentNavPoint);

  // update the relative position of the nav point
  pn_r = nav.getPnr(currentNavPoint, myRobotPose);

  // are we close enough to our currentNavPoint?
  bool areWeThereYet = nav.closeEnough(myRobotPose, currentNavPoint);

  // use navpoint and robot pose to calculate PID changes needed and modify motor output
  motors.update(pn_r, areWeThereYet);

}

uint32_t btDebugCallback(uint32_t currentTime) {
    if (serialDebug == true){
        float currentDist = nav.getDistanceRelRobot(pn_r);
        float desiredHeading = nav.getHeadingRelRobot(pn_r);
        float x = currentNavPoint.x;
        float y = currentNavPoint.y;
        String outputBuf = (String)millis() + "," + 
                            (String)x + "," + 
                            (String)y + "," + 
                            (String)currentDist + "," + 
                            (String)motors.velocity  + "," + 
                            (String)desiredHeading  + "," + 
                            (String)motors.angleAdj + ";";
        Serial.println(outputBuf);
    }
  

  return (currentTime + CORE_TICK_RATE * 2000);
}

uint32_t updateCallback(uint32_t currentTime) {
  if (robotState == ATTACK){
    // find nearest ball
    NavPoint nearestBall = nav.findNearestBall();
    goalPoint = nav.findNearestBall();
  }
  return (currentTime + CORE_TICK_RATE * 500);
}
