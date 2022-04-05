#include "navlist.h"
#include "btserial.h"
//      navPoint nav[10];
//    unsigned int currNav = 0;
//    unsigned int maxNav = 4;


void navList::init(navPoint navPoints[]){
  Serial.print("navlist");
  currNav = -1;
  maxNav = 4;
}


navPoint navList::getNextNavPoint(){

    currNav += 1;
    if (currNav > maxNav){
      /// just start over again
      currNav = 0;
    }
    navPoint newNavPoint  = nav[currNav];
    BTSerial.print(newNavPoint.x);
    BTSerial.print(", ");
    BTSerial.println(newNavPoint.y);
    

    return newNavPoint;
  }
  
void navList::addNavPoint(navPoint navpoint){
  // nothing
  Serial.print("navList::addNavPoint()");
}
