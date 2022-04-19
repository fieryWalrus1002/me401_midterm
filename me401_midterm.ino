#include "main.h"
//#include "ME401_Radio.h"
#include "ME401_PID.h"
#include "btserial.h"
#include "irDistance.h"
#include "navsystem.h"
#include "newRadio.h"


void attack(){
    // this sets the goalpoint we are trying to get to. this is used during the checkPath function to set currentNavPoint
    goalPoint = nav.findNearestBall(); 
    // get the position of the nearest ball
//    NavPoint pnr = nav.getPnr(goalPoint , myRobotPose);h
   
//   if(pnr.x > 0 && pnr.x < 500 && pnr.y > -100 && pnr.y < 100){
//      // if the ball is close enough, open the gate
//      openGate(true);
//  
//      // and increment our score counter
//      nav.CountBalls();
//   }
//   else{
//    // if there isn't a ball right in front of us, close the gate
//      openGate(false);
//   }
}

void defend(){
//  Serial.println("DEFEND"); 
  goalPoint = home_base;
}

void capture(){
  digitalWrite(CAPTURE_PIN, HIGH);
  digitalWrite(ATTACK_PIN, LOW);
  // set our goalPoint to home base
  goalPoint = home_base;

  // check if we have reached our base
  bool baseReached = nav.closeEnough(myRobotPose, goalPoint);

  if (baseReached == true){
    //drop the payload
    nav.depositTheCash();

    // reset state to ATTACK so we can go for more balls
    robotState = ATTACK;
    ballcaptured = 0;
  }
  
}

void checkStatus(){
    
    // update all robot, ball and global obstacle positions with radio data
    comms.updateRobotPoseAndBallPositions();

    // assign our robot's information to a robotPose so we can use it easily
    myRobotPose = comms.getRobotPose(MY_ROBOT_ID);

    // update the navigation systems data
    //    nav.update(&currentNavPoint);
   
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
if (ballcaptured >= 6)
{
  robotState = CAPTURE;
}
}



void setup() {
  Serial.begin(115200);
  Serial.print("serial begin");
  pinMode(ATTACK_PIN, OUTPUT);
  pinMode(CAPTURE_PIN, OUTPUT);
  pinMode(SUNBEAM_SERVO_PIN, OUTPUT);
  irSensor.init(); // Sharp IR distance sensor initialize
  Serial.print(", ir sensor online");
  
  if (RADIO == true){
    comms.ME401_Radio_initialize(); // Initialize the RFM69HCW radio  
    checkStatus();
    Serial.print(", radio online");
  } else {
    Serial.print(", offline mode");
  }

  initBtSerial();
  Serial.println(", btserial online");

  // attach motor servo pins so they actually work when we command them
  motors.leftServo.attach(leftServoPin);
  motors.rightServo.attach(rightServoPin);
  gateServo.attach(GATE_SERVO_PIN);
  sunbeamServo.attach(SUNBEAM_SERVO_PIN);
  // set the home base coordinates to the corner of the arena we start in
  nav.setHomeBase(robotPoses[MY_ROBOT_ID]);
  Serial.print(", home_base set to (");
  Serial.print(home_base.x);
  Serial.print(", ");
  Serial.print(home_base.y);
  Serial.print(")");
  
  // Initialize the PID and IR interrupts
  // TODO: Change the kp, ki, kd in the ME491_PID_IR.h file to match your new tunings
  //       that you did after installing the sensor on your robot
//  setupPIDandIR();
  
  // serial output every 2s
  Serial.println("Serial output every 2s on ");
  attachCoreTimerService(btDebugCallback);
  
//  // update the goal position every 500 ms
  attachCoreTimerService(updateCallback);
//  Serial.println("coreTimer attached");

  // attach the external interrupts for the limit switches
  attachInterrupt(L_LIMIT_EXTINT, handleCrashL, FALLING);
  attachInterrupt(R_LIMIT_EXTINT, handleCrashR, FALLING);
  Serial.println("limit switches activated");
}




void loop() {
  static long prevTime = 0;
  long timeElapsed = millis() - prevTime;
  prevTime = millis();
  
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
   } else {
//      comms.getTestPoses();
      handleState();
      myRobotPose.x = 1000;
      myRobotPose.y = 1000;
      currentNavPoint.x = 1200;
      currentNavPoint.y = 1000;
      changeState();

   }

   /***********************************************************************
   *  Collision avoidance
   **********************************************************************/


   // check for upcoming collision with local obstacles
   pathClear = irSensor.checkFrontIrDistance(lookAheadDistance);
   if (pathClear == false){
    
//    pathAngle = irSensor.scanForPath();
//    nav.setPathByAngle(pathAngle);
      CRASH_FLAG = true;
   }

   // resolve any crashes with local obstacles that have occured
   while (CRASH_FLAG == true && IMMOBILE == false){      
      CRASH_FLAG = nav.crashState(CRASH_SIDE);
      Serial.print("CRASH");
   } 

   
//
//  // periodic path checking for global obstacles
//  if (updateState = true){
//      // check to see if we have a clear path to our goal point
//      // if not, we will adjust our course to the side of an obstacle
//      nav.checkPathToGoal(&currentNavPoint);
//  
//      updateState = false;
//  }

  /***********************************************************************
   *  Motor updates
   **********************************************************************/

  
  nav.editNavPoint(&currentNavPoint,  1200,  1000);
  
  // update the relative position of the nav point
  pn_r = nav.getPnr(currentNavPoint, myRobotPose);
  
  // are we close enough to our currentNavPoint? if so, motor output will be zero for this loop
  bool areWeThereYet = nav.closeEnough(myRobotPose, currentNavPoint);

  // use navpoint and robot pose to calculate PID changes needed and modify motor output
  motors.update(pn_r, areWeThereYet); 
   
} // end loop()

/***********************************************************************
 * CALLBACK FUNCTIONS
 **********************************************************************/

uint32_t btDebugCallback(uint32_t currentTime) {
  String outputBuf;
     if (btDebug == true || serialDebug == true){
      float currentDist = nav.getDistanceRelRobot(pn_r);
      float desiredHeading = nav.getHeadingRelRobot(pn_r);
      float x = currentNavPoint.x;
      float y = currentNavPoint.y;
      outputBuf = (String)millis() + "," +
                          "RobotState: " +
                          (String)robotState + "," +
                          (String)x + "," + 
                          (String)y + "," + 
                          (String)currentDist + "," + 
                          (String)motors.velocity  + "," + 
                          (String)desiredHeading  + "," + 
                          (String)motors.angleAdj + ";";
     }
      if (btDebug == true){
        BTSerial.println(outputBuf);
      } 
      if (serialDebug == true){
        Serial.println(outputBuf);
      }

  return (currentTime + CORE_TICK_RATE * 2000);
}

uint32_t updateCallback(uint32_t currentTime) {
  /* Periodically set the flag to require the calculation of path around global obstacles.
   * Global obstacles don't move, or in the case of robots, don't move very fast. There
   * is no reason to update this every single loop. 
    */
  updateState = true;
  return (currentTime + CORE_TICK_RATE * 250);
}

void handleCrashL(){
  // interrupt handler for limit switch external interrupt on left side
  CRASH_FLAG = true;
  CRASH_SIDE = 0;
}

void handleCrashR(){
  // interrupt handler for limit switch external interrupt on right side
  CRASH_FLAG = true;
  CRASH_SIDE = 1;
}
