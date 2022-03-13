/*  
 *  propulsion motor PID controller
 *  Written by Magnus Wood (magnus.wood@wsu.edu) on 3-12-22
 *  For ME401 Spring 22
 *  
 *  J18 and J17 Servo connectors
 *  chipKit Servo Motor Driver
 *  Pin# 
 *  26   A: PMD0/RE0   
 *  27   B: PMD1/RE1   
 *  28  C: PMD2/RE2   
 *  29  D: PMD3/RE3
 * 
 *  Power supplied to J1 from J14
 *  Reference Manual located at:
 *  https://digilent.com/reference/add-ons/motor-shield/reference-manual
*/

#ifndef _MOTORPID_H_
#def _MOTORPID_H_

typedef struct {
  // PID controller params
  double Kp;
  double Ki;
  double Kd;
  
  // stuff we need to keep track of
  double _integral;
  double _prevError;
  double _dt;
  
  // max output limit for the controller
  double maxLimit;
  double minLimit;
} PIDVars;


double pidCalc(PIDVars *var, double currentError){
  var->_integral += currentError*(vars->_dt);
  
  double output = (vars->Kp)*currentError + (vars->Ki)*(vars->_integral) + (vars->Kd)*((currentError - (vars->_prevError)) / (vars->_dt);
  
  vars->_prev_err = current _err;
  
  if (output >(vars->maxLimit))
    output = vars->maxLimit;
  else if (output < (vars->minLimit))
    output = vars->minLimit;
    
  return output;


     
#endif
