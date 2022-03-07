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
#define HOMERADIUS 150

struct HOMEBASE {
  float x = 1000;
  float y = 1000;
  float radius = HOMERADIUS;
} home_base;

void init_nav(int){
  home_base.x = 0.0;
  home_base.y = 0.0;
}

#endif
