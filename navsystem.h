    /*  
    *  Navigation systems and functions
    *  Written by Magnus Wood (magnus.wood@wsu.edu) on 4-4-22
    *  Modified 4-6-22
    *  For ME401 Spring 22 miderm robot project
    * 
    *  Structs: 
    *  - NavPoint: a pair of xy coordinates
    *  
    *  Variables:
    *   - home_base: a NavPoint corresponding to the robots home base location, used for 
    *              returning a captured ball.
    *   - test_course: an array of NavPoints, for testing a path following behavior. 
    *   - nav: an object of class NavSystem, for utilizing the nav functions.
    *   - navList: an array of NavPoints I am using to store multiple NavPoints to 
    *       define a path. 
    *   - currNav, maxNav: variables that help index the lenght of navList, ensuring
    *       that undefined behavior doesn't occur when iterating through the array. 
    *   - currentNavPoint: a variable that holds the current goal position of the robot.
    *       We modify the robot's destination by assigning new x/y coordinates to this 
    *       single navPoint. 
    * 
    *  Classes:
    *   NavSystem: The NavSystem class holds a number of variables and functions used 
    *   for navigation of the robot, including helper functions for calculating angles, 
    *   and NavPoint edit functions to change coordinates.
    * 
    *  Usage:
    *  - setHomeBase(currentRobotPose): called during setup, changes the coordinates of home_base NavPoint 
    *              to the corner of the quadrant that the robot starts up in. 
    * 
    *  - getPnr(&currentNavTarget): get position of NavPoint wrt robot frame. Uses navpoint (can be ball, 
    *            whatever, just a point in space) and robot pose to calculate the position 
    *          of the navpoint in relation to the robot frame.
    *           Returns navPoint with coordinates of the P in relation to robot frame.
    *
    *   - update(&currentNavTarget): Called every loop as part of checkStatus(), checks if the waypointreached
    *            flag is true. If it is true, then get the next NavPoint coordinates and 
    *            reset flag.
    *
    *   - getHeadingRelRobot(p_nr): Given a position of a NavPoint relative to the 
    *       robot's position, returns the heading in radians wrt the robot.
    * 
    *   - getDistanceRelRobot(p_nr): Given a position of a NavPoint relative to the
    *       robot's position, return the Euclidean distance in millimeters in relation
    *       to the robot. 
    * 
    *   - getNextNavPoint(&currentNavTarget): Given the current NavPoint pointer, 
    *       assign the coordinates of the next NavPoint in the series to it. 
    *   
    *   - convDegRads/ convRadsDeg(): helper functions I used for troubleshooting. Does
    *       simple conversion between rads and degrees and back again for testing.
    * 
    *   - addNavPoint(): stub, doesn't do anything.
    * 
    *   - goToPoint(&currentNavTarget, int): like editNavPoint, it takes the pointer for
    *       the currentNavTarget and modifies the x any y values. However, instead of
    *       geting those values from a NavPoint in an array, it takes an integer and 
    *       turns it into coordinates. This is mostly used in the btserial as a 
    *       convenience, so that destinations can be sent over serial for remote 
    *       control purposes. 
    */

 
#ifndef _NAVSYSTEM_H_
#define _NAVSYSTEM_H_
// #include "navlist.h"
#include "newRadio.h"
//#include "ME401_Radio.h"
#define M_PI 3.14159

class NavPoint {
    public:
        NavPoint(float, float);
        float x; 
        float y;
        NavPoint();
};

class NavSystem
{  
  public:
    NavPoint currentNavPoint = {1000.0, 1000.0};
    void update(NavPoint*);
    void setHomeBase(RobotPose);
    bool checkWaypointStatus();
    void editNavPoint(NavPoint*, float, float);
    NavPoint getPnr(NavPoint, RobotPose);
    double getHeadingRelRobot(NavPoint);
    double getDistanceRelRobot(NavPoint);
    void goToPoint(int, NavPoint*);
    void getNextNavPoint(NavPoint*); //pass currentNavPoint to getNext and it will assign the new coordinates
    void addNavPoint(NavPoint);
    NavPoint getPnw(NavPoint navpoint, RobotPose robot);
    void checkPath(NavPoint* currentNavPoint);

  private:
    int currNav = 0;
    int maxNav = 10;
    bool waypointReached = false;
    double convDegRads(float);
    double convRadDegs(float);
};

NavPoint navList[10] = {{500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0},
                        {500., 500.0}
                        };

NavPoint currentNavPoint = {1000.0, 1000.0};
int obsOffset = 50;
// assign goalPoint a position when have a ball as a target, or the home base when in capture
NavPoint goalPoint = {1000.0, 1000.0};
NavPoint home_base = {0.0, 0.0};

NavSystem nav;

#endif /* _NAVSYSTEM_H_ */
