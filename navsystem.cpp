/*  
 *  Navigation systems and functions
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 4-4-22
 *  For ME401 Spring 22
 *  
 *  Usage:
 *  .
 *  
 *  
 *   
*/


#include "navsystem.h"
#include <Arduino.h>

#include "btserial.h"

NavPoint::NavPoint(float x1, float y1){
    x = x1;
    y = y1;
}

NavPoint::NavPoint(){
    x = 0.0;
    y = 0.0;
}


void NavSystem::update(NavPoint *cnp){
    if (waypointReached == true){
        getNextNavPoint(cnp);
        waypointReached = false;
    }

}



void NavSystem::editNavPoint(NavPoint *nextPoint, float x, float y){
  nextPoint->x = x;
  nextPoint->y = y;
}


NavPoint home_base;

NavPoint testCourse[] = {
                          {250.0, 250.0}, 
                          {1750.0, 250.0}, 
                          {1750.0, 1750.0}, 
                          {250.0, 1750.0},
                        };

void NavSystem::setHomeBase(RobotPose myStartPose){
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


NavPoint NavSystem::getPnr(NavPoint navpoint, RobotPose robot){
    /* get position of NavPoint wrt robot frame
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

      // define new NavPoint
      double px = x1 - x1t;
      double py = y1 - y1t;
      NavPoint pnr = {px, py};
      
      return pnr;
}

double NavSystem::getHeadingRelRobot(NavPoint pn_r){
      // get heading from robot to relative navpoint in rads
      double hRads = atan2(pn_r.y, pn_r.x);
      return hRads;
}

double NavSystem::getDistanceRelRobot(NavPoint pn_r){
  /* Calculate the distance from a navpoint relative to the robot
   */
   double distance = sqrt(pn_r.x * pn_r.x + pn_r.y * pn_r.y);
   
   return distance;
}

double NavSystem::convDegRads(float degree){
  double radian = degree * (M_PI / 180.0);
  return radian;
}

double NavSystem::convRadDegs(float radian){
  double degree = radian / (M_PI / 180.0);
  return degree;
}


void NavSystem::goToPoint(int xy, NavPoint* currentNavPoint){
   // 1020.0140 / 10000 = 
    float x = xy / 10000;
    float y = xy - (x * 10000);
    
    currentNavPoint->x = x;
    currentNavPoint->y = y;
}



void NavSystem::getNextNavPoint(NavPoint *oldNavPoint){

    currNav += 1;
    if (currNav > maxNav){
      /// just start over again
      currNav = 0;
    }
    NavPoint newNavPoint = navList[currNav];
    oldNavPoint->x = newNavPoint.x;
    oldNavPoint->y = newNavPoint.y;
    if (serialDebug == true){
        BTSerial.print(oldNavPoint->x);
        BTSerial.print(", ");
        BTSerial.println(oldNavPoint->y);
    }

  }
  
void NavSystem::addNavPoint(NavPoint navpoint){
}



