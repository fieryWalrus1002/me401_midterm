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
//  Serial.println("ATTACK");
//  BallPosition closestballPos = ballPositions[0];
  nav.editNavPoint(&currentNavPoint,ballPositions[0].x,ballPositions[0].y);
}

void defend(){
//  Serial.println("DEFEND"); 
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
  attachCoreTimerService(btDebugCallback);
  motors.leftServo.attach(leftServoPin);
  motors.rightServo.attach(rightServoPin);
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


//  while (blocked == true):
//  float bestAngle = distanceSweep();
//
//  NavPoint avoidancePoint = findPath(bestAngle); 
//    pick a point out in front of the orbot, at that angle, and return that point. 
//    
//  editNavPoint(&currentNavPoint, avoidancePoint.x, avoidancePoint.y);
//
//  if (wayPointreached == true){
//    blocked = false;
//    editNavPoint(^current, goal.x, goal.y);
//  }
  
  
//   // set the time in seconds
//   dT = (millis() - lastTime) / 1000;
 
  // update the environment and change states as required
   if (RADIO == true){
      checkStatus(); // check the status of the game environment
      changeState(); // check to see if a state change is called for
      handleState(); // execute current state function
   }

  // update the relative position of the nav point
  pn_r = nav.getPnr(currentNavPoint, myRobotPose);

  // use navpoint and robot pose to calculate PID changes needed and modify motor output
  motors.update(pn_r, serialDebug);

//   lastdT = dT;
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
        BTSerial.println(outputBuf);
    }
  

  return (currentTime + CORE_TICK_RATE * 2000);
}
