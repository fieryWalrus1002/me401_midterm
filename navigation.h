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

#define DT 50 // length of one step of movement time

typedef struct navPoint {
  float x; 
  float y;
};

navPoint home_base;

navPoint testCourse[] = {
                          {250.0, 250.0}, 
                          {1750.0, 250.0}, 
                          {1750.0, 1750.0}, 
                          {250.0, 1750.0},
};   

double calcDistError(navPoint navpoint, RobotPose pose){
  double distErr = sqrt(pow(abs(pose.x - navpoint.x), 2) + pow(abs(pose.y - navpoint.y), 2));
  return distErr;
}




void robotBlocked(){
//    moveRobot(ALLSTOP);
//    delay(100);
//    moveRobot(BACKUP);
//    delay(200);
//    moveRobot(TURNLEFT);
//    delay(200);
}

int chartCourse(){
  int lDist = getDistance(60);
  int cDist = getDistance(30);
  int rDist = getDistance(0);

  if (lDist > 15 && cDist > 15 && rDist > 15){
    return 0; // keep going
  }

  if (lDist > rDist){
    if (lDist > 15 && cDist > 15){
      return 1; // turn left
    } else if (lDist > 15 && cDist < 15){
      return 2; // back up and turn left
    } else {
      return 3; // back up a lot and turn left
    }
  }
  
  if (lDist < rDist){
    if (rDist > 15 && cDist > 15){
      return 4; // turn right
    } else if (rDist > 15 && cDist < 15){
      return 5; // back up and turn right
    } else {
      return 6; // back up a lot and turn right
    }
  }
}



void avoidanceTest(){
//  int course = chartCourse();
//
//  switch (course) {
//    case 0:
//      // no obstruction, keep on moving on
//      moveRobot(FORWARD);
//      break;
//    case 1:
//      // turn left
//       moveRobot(TURNLEFT);
//       delay(DT);
//      break;
//    case 2:
//      // back up and turn left
//      moveRobot(BACKUP);
//      delay(DT * 2);
//      moveRobot(TURNLEFT);
//      delay(DT);
//      break;    
//    case 3:
//      // back up a lot and turn left
//      moveRobot(BACKUP);
//      delay(DT * 5);
//      moveRobot(TURNLEFT);
//      delay(DT);
//      break;
//     case 4:
//      // turn right
//      moveRobot(TURNRIGHT);
//      delay(DT);
//      break;
//    case 5:
//      // back up and turn right
//      moveRobot(BACKUP);
//      delay(DT * 2);
//      moveRobot(TURNRIGHT);
//      delay(DT);
//      break;
//    case 6:
//        // back up a lot and turn right
//      moveRobot(BACKUP);
//      delay(DT * 5);
//      moveRobot(TURNRIGHT);
//      delay(DT);
//      break;
//    default:
//      break;
//  }
}

#endif
