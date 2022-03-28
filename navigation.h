/*  
 *  robot navigation code
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-4-22
 *  For ME401 Spring 22
 *  
 *  defines home base on startup
*/

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_
// home area is quarter circle, area to be defined
#define HOMERADIUS 50
#define M_PI 3.14159
#define DT 50 // length of one step of movement time
#include "main.h"

typedef struct navPoint {
  float x; 
  float y;
};



// function prototypes to make the compiler quit complaining
void setHomeBase(RobotPose);
void editNavPoint(navPoint, float, float);

navPoint currentNavPoint = {2000.0, 500.0};

void assignNavPointValues(int value){
  // take an int, and derive coordinates from it
  
}

void editNavPoint(navPoint *nextPoint, float x, float y){
  nextPoint->x = x;
  nextPoint->y = y;
}



typedef struct navList {
  int currNav = 0;
  int maxNav = 4;
  navPoint nav[10] = {
                        {250.0, 250.0}, 
                        {1750.0, 250.0}, 
                        {1750.0, 1750.0}, 
                        {250.0, 1750.0},
                       };   
} navlist;

navPoint home_base;

navPoint testCourse[] = {
                          {250.0, 250.0}, 
                          {1750.0, 250.0}, 
                          {1750.0, 1750.0}, 
                          {250.0, 1750.0},
                        };   


void initNavSystem(RobotPose *currentPose){
    setHomeBase(*currentPose); // sets home base location based on what quadrant you start the robot in
    float x = currentPose->x;
    float y = currentPose->y;
    editNavPoint(&currentNavPoint, x, y); // then hand it to the robot and change currentNavPoint to our current position 
}

void setHomeBase(RobotPose myStartPose){
  // automatically set your home base coordinates according to what quadrant your robot starts up in
  // TODO: this is not finished you need to actually measure this shit for the actual locations
  
  if (myStartPose.x < 1100){
    if (myStartPose.y > 1100){
      // closest to door into room
      home_base.x = 140;
      home_base.y = 2050;
    } else {
      // closest to origin
      home_base.x = 140;
      home_base.y = 140;
    }
  if (myStartPose.x > 1100){
    if (myStartPose.y > 1100){
      // kitty corner from origin
      home_base.x = 2050;
      home_base.y = 2050;
    } else {
      // that last dumb corner without a clever name
      home_base.x = 2050;
      home_base.y = 140;
    }
  }
  }
}
//
//navPoint getNextNavPoint(){
//  // TODO: better navPoint buffer for actual use. This is just for a square pattern test.
//  int currNav = navlist.currNav;
//  navPoint newNavPoint  = navlist.nav[currNav];
//  currNav += 1;
//  if (currNav > navlist.maxNav){
//    /// just start over again
//    navlist.currNav = 0;
//  }
//  return newNavPoint;
//}

navPoint getPnr(navPoint navpoint, RobotPose robot){
    /* get position of navPoint wrt robot frame
     *  Uses navpoint (can be ball, whatever, just a point in space) and robot pose to
     *  calculate the position of the navpoint in relation to the robot frame
     *  Returns a navpoint of the navpoint in relation to the robot frame. 
     */
      double theta = (float)robot.theta / 1000.0; // convert from the int16_t format to double
      double c = cos(theta);
      double s = sin(theta);

      // Pnr = Rrw' * Pnw  - Rrw' * Prw
      // rotation matrix
      double x1 = c * navpoint.x + s * navpoint.y;
      double y1 = -s * navpoint.x + c * navpoint.y;

      // translation matrix
      double x1t = c * robot.x + s * robot.y;
      double y1t = -s * robot.x + c * robot.y;

      // define new navPoint
      double px = x1 - x1t;
      double py = y1 - y1t;
      navPoint pnr = {px, py};
      
      return pnr;
}

double getHeadingRelRobot(navPoint pn_r){
      // get heading from robot to relative navpoint in rads
      double hRads = atan2(pn_r.y, pn_r.x);
      return hRads;
}

double getDistanceRelRobot(navPoint pn_r){
  /* Calculate the distance from a navpoint relative to the robot
   */
   double distance = sqrt(pn_r.x * pn_r.x + pn_r.y * pn_r.y);
   
   return distance;
}

double convDegRads(float degree){
  double radian = degree * (M_PI / 180.0);
  return radian;
}

double convRadDegs(float radian){
  double degree = radian / (M_PI / 180.0);
  return degree;
}

void testRotMatStuff(){
  double rot = convRadDegs(90);
  RobotPose robot = {true, 2, 1678, 623, 1500};
  navPoint testCourse[] = {           
                            {901.0, 719.0},
                            {0.0, 0.0}, 
                            {8.0, 2100.0},
  };   

  for (int i = 0; i < 3; i++){
    // calculate distance and heading to a point relative to the robot
    navPoint currentNav = testCourse[i];
    navPoint pn_r = getPnr(currentNav, robot);
    double distance = getDistanceRelRobot(pn_r);
    double headToNav = getHeadingRelRobot(pn_r);
    double headingDegrees = convRadDegs(headToNav);
//    Serial.println(i + 1);
//    Serial.print("(");
//    Serial.print(currentNav.x);
//    Serial.print(", ");
//    Serial.print(currentNav.y);
//    Serial.print("), (");
//    Serial.print(pn_r.x);
//    Serial.print(", ");
//    Serial.print(pn_r.y);
//    Serial.println(")");
//    Serial.print("h:[");
//    Serial.print(headingDegrees);
//    Serial.println("]");
  }
}

void goToPoint(int xy, navPoint* currentNavPoint){
   // 1020.0140 / 10000 = 
    int x = xy / 10000;
    int y = xy - x * 10000;

    currentNavPoint->x = x;
    currentNavPoint->y = y;

}


#endif
