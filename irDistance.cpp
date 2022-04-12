#include "irDistance.h"

void IrSensor::init(){
  servo.attach(SERVO_PIN);
  pinMode(IR_OUT, INPUT);
}

double IrSensor::scanAreaForGap(){
  // Korey's scan algorithm
  //Scan the area around the obstacle to search for routes around it if any
  //Seek ‘empty’ direction, if two open paths go right
  // so the scanAreaForGap should return a double, corresponding to an angle from the robot that looks more open
  int minAngle = -90; // I'm making these up, I haven't used the servo yet
  int maxAngle = 90;
  double bestAngle = 0; // holds the best angle corresponding to the highest distance
  int highestDistance = 0; // the best really sucks to begin with so anything it finds is okay
  int stupidCloseWeAreStillBlocked = 400; // how far away is stupid close? Change it to something sensible
  
  for (int i = minAngle; i < maxAngle; i++){
    // this code sucks because its going to iterate through ALL the angles from -90 to 90. Maybe a set number of angles to check?
    // or have the angle be an i value multiplied by something? i don't know here is i * 30.
    int angle = i * 30;
    int obsDist = getDistance(angle); // get the distance at this angle
    
    if (obsDist < highestDistance){
      // if the measured distance is more than our current highest, its more open so we mark it as our current bestAngle
      highestDistance = obsDist;
      bestAngle = angle;
    }
  }

  if (highestDistance < stupidCloseWeAreStillBlocked){
    // uh oh we're really close on even the best one, so I guess we're still blocked
    // tell it that the best angle is straight backwards
    return -180;
  } else {
    // if the best angle is open enough, ie greater than our stupid close variable, 
    return bestAngle;
  }
};

void IrSensor::calibrateIrSensor(){
  // mean of 10 values for each distance
  for (int i = 0; i < 10; i++){
    int sumVal = 0;
    Serial.print(calDist[i]);
    delay(4000);
    Serial.print(", ");
    delay(1000);
    for (int j = 0; j<10; j++){
      sumVal += getRawDistance(90);
      delay(100);
    }

    Serial.println((float)sumVal / 10.0);
  }
}


void IrSensor::moveIrSensor(int desiredAngle){
  // the servo library angle doesn't really match any real degrees so we map our desired angle to a calibrated set of values
  static int lastTheta = 0;
  int theta = map(desiredAngle, 0, SERVOLIMIT, 30, 125);
  servo.write(theta);
  delay(abs(theta - lastTheta)*5); // wait for servo to move before continuing, dependent on distance to travel
  lastTheta = theta; 
}
  
int IrSensor::getRawDistance(int theta){
  moveIrSensor(theta);
  int sumVal = 0;
  for (int i = 0; i < SAMPLE_NUM; i++){
    sumVal += analogRead(IR_OUT); 
  }
  int irVal = sumVal / SAMPLE_NUM;
  return irVal;
}

int IrSensor::getDistance(int theta){
  int irVal = getRawDistance(theta);
  int calDist = polyFit(irVal);
  return calDist;
}

int IrSensor::powerFit(int irVal){
   // y = 13722x-1.106
  float calDist = 13722.0 * pow((float)irVal, -1.106);
  return (int)calDist;
}

int IrSensor::polyFit(int irVal){
  //y = -3E-07x3 + 0.0007x2 - 0.5821x + 173.8
  
  float rawVal = (float)irVal;
  float calDist = -0.0000003 * pow(rawVal, 3) + 0.0007 * pow(rawVal, 2) -0.5821 * rawVal + 173.8;
  return (int)calDist;
}

float IrSensor::getRadHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float radHeading = ((float)bestTheta - (float)SERVOLIMIT / 2.0) * 0.01745;
  return radHeading; 
}

float IrSensor::getDegreeHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float degreeHeading = (float)bestTheta - ((float)SERVOLIMIT / 2.0);
  return degreeHeading;
}


float IrSensor::distanceSweep(){
  static bool rightToLeft = 1;
  int numMeas = SERVO_INCREMENT;
  int maxDistAngle = 0;
  int maxDist = 0;
  int sweepVal[numMeas];
  int angleInc = SERVOLIMIT / (numMeas - 1);

  if (rightToLeft == 1){
      // sweep from min to max angle
    for (int i = 0; i < numMeas; i++){
      int distMeas = getDistance(i * angleInc);
      sweepVal[i] = distMeas;
    }
    rightToLeft = 0; // toggle for next run
  } else {
    // sweep from max angle back to zero
    for (int i = numMeas; i >= 0; i--){
      int distMeas = getDistance(i * angleInc);
      sweepVal[i] = distMeas;
    }
     rightToLeft = 1; // toggle for next run
  }

   // find the angle to the greatest open space
  for (int i = 0; i < numMeas; i++) {
    if (sweepVal[i] > maxDist){
      maxDist = sweepVal[i];
      maxDistAngle =  i * angleInc;
    }
  }
  Serial.print("maxDistAngle: ");
  Serial.println(maxDistAngle);

  // return the most open heading in either degrees or radians
  if (RADIANS == 0){
    return getDegreeHeading(maxDistAngle);
  } else
  {      
    return getRadHeading(maxDistAngle);   
  }
}
