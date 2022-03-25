/*  
 *  propulsion motor PID controller
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-12-22
 *  For ME401 Spring 22
 *  
 *  Reference Manual located at:
 *  https://digilent.com/reference/add-ons/motor-shield/reference-manual
*/

#ifndef _MOTORPID_H_
#define _MOTORPID_H_

float const ERRDECAY = 0.1; // rate of integral decay

typedef struct {
  // PID controller params
  double Kp;
  double Ki;
  double Kd;
  
  // stuff we need to keep track of
  double _integral;
  double _prevError;
  double _dT;
  
  // max output limit for the controller
  double maxLimit;
  double minLimit;
} PIDVars;

double pidCalc(PIDVars *vars, double currentError, bool debug){
  vars->_integral += vars->_integral * (1 - ERRDECAY) + currentError; // integral error weighted towards current error, old error values less weight

  double propError = (vars->Kp) * currentError;
//  float dampError = (vars->Kd) * ((currentError - (vars->_prevError))); // derivative of last time step
//  float intError = (vars->Ki) * vars->_integral;
//  
  double dampError = (vars->Kd) * ((currentError - (vars->_prevError))); // derivative of last time step
  double intError = (vars->Ki) * vars->_integral;
  double output = propError + dampError + intError;
  
  vars->_prevError = currentError;
  
  if (output >(vars->maxLimit))
    output = vars->maxLimit;
  else if (output < (vars->minLimit))
    output = vars->minLimit;

  if (debug == true){
     Serial.print("Kp: ");
    Serial.print(vars->Kp);
    Serial.print(", error: ");
    Serial.print(currentError);
    Serial.print(", Ki: ");
    Serial.print(vars->Ki);
    Serial.print(", integral: ");
    Serial.print(vars->_integral);
    Serial.print(", Kd: ");
    Serial.print(vars->Kd);
    Serial.print(", damper: ");
    Serial.print(dampError);
    Serial.print(", output: ");
    Serial.println(output);  
  }

  return output;
}
     
#endif
