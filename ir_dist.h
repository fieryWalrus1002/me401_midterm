/*  
 *  IR Distance Calculation Library
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-4-22
 *  For ME401 Spring 22
 *  
 *  Hardware:
 *  Pololu Carrier With Sharp GP2Y0A60SZLF Analog Distance Sensor
 *  SG90 servo
 *  
 *  Inspiration on calibrating sensor:
 *  https://www.instructables.com/How-to-setup-a-Pololu-Carrier-with-Sharp-GP2Y0A60S/
 *  https://blogs.ntu.edu.sg/scemdp-201415s2-g12/infrared-sensor-and-its-calibration/
 *  
 *  Operation:
 *  While power is on, IR pulses are constantly pulsing. analogRead(IR_OUT) will get
 *  analog value, larger is closer. Sensor is 50-300mm, but is non-linear and must be calibrated.
 *  Calibration values can be stored in this file, 
 *  
 *  Calibration:
 *  Light falls off with the square of the distance, so by testing intensity at known distances, 
 *  relationship between 1/analogRead(IR_OUT) and the distance should be linear. Use calibrateSensor()
 *  to print out serial measurements for calibration.
 *  
*/

#ifndef _IR_DIST_H_
#define _IR_DIST_H_
#include <Servo.h>

#define IR_OUT A9
#define SERVO_PIN 30
#define RADIANS 1 // if 1, angle measures will be returned in radian headings (+-)
#define SERVOLIMIT 60 // max angle of the servo sweep
#define SERVO_INCREMENT 10 // number of increments to divide the servo into for IR distance measurements
#define SAMPLE_NUM 3 // how many IR measurements to average over

int getDistance(int theta);
void moveIrSensor(int theta);
int getRawDistance(int theta);
int polyFit(int irVal);
int powerFit(int irVal);

int calDist[10] = {5, 10, 15, 20, 25, 30, 40, 50, 60, 70};
int sweepArray[12] = {0, 30, 60, 90, 120, 150, 180, 150, 120, 90, 60, 30};


Servo servo;

void initIrSensor(){
  servo.attach(SERVO_PIN);
  pinMode(IR_OUT, INPUT);
  moveIrSensor(SERVOLIMIT / 2);
  delay(5000);
}

void calibrateIrSensor(){
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

void moveIrSensor(int desiredAngle){
  // the servo library angle doesn't really match any real degrees so we map our desired angle to a calibrated set of values
  static int lastTheta = 0;
  int theta = map(desiredAngle, 0, SERVOLIMIT, 30, 125);
  servo.write(theta);
  delay(abs(theta - lastTheta)*5); // wait for servo to move before continuing, dependent on distance to travel
  lastTheta = theta; 
}
  
int getRawDistance(int theta){
  moveIrSensor(theta);
  int sumVal = 0;
  for (int i = 0; i < SAMPLE_NUM; i++){
    sumVal += analogRead(IR_OUT); 
    delay(10);
  }
  int irVal = sumVal / SAMPLE_NUM;
  return irVal;
}

int getDistance(int theta){
  int irVal = getRawDistance(theta);
  int calDist = polyFit(irVal);
  return calDist;
}

int powerFit(int irVal){
   // y = 13722x-1.106
  float calDist = 13722.0 * pow((float)irVal, -1.106);
  return (int)calDist;
}

int polyFit(int irVal){
  //y = -3E-07x3 + 0.0007x2 - 0.5821x + 173.8
  
  float rawVal = (float)irVal;
  float calDist = -0.0000003 * pow(rawVal, 3) + 0.0007 * pow(rawVal, 2) -0.5821 * rawVal + 173.8;
  return (int)calDist;
}

float getRadHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float radHeading = ((float)bestTheta - (float)SERVOLIMIT / 2.0) * 0.01745;
  return radHeading; 
}

float getDegreeHeading(int bestTheta)
{
  // theta is initially in degrees, and an int
  float degreeHeading = (float)bestTheta - ((float)SERVOLIMIT / 2.0);
  return degreeHeading;
}


float distanceSweep(){
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

void testGetDistance(){
  Serial.println(getDistance(90));
  delay(500);
}

 
#endif
