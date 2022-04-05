/*  
 *  NavSystem
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 4-2-22
 *  For ME401 Spring 22
 *  
 * 
 *  
 */
 
#ifndef _NAVSYSTEM_H_
#define _NAVSYSTEM_H_
// #include "navlist.h"
#include "ME401_Radio.h"
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

NavSystem nav;

#endif /* _NAVSYSTEM_H_ */