#include "PID.h"

double PID_update(double setValue, double processValue, struct PID_parameters *PID){
  
  double errorValue = setValue - processValue;

  double Proportional = errorValue;
  
  double Integral = PID->integralValue + errorValue*PID->dt;
  
  double Derivative = (PID->alpha * PID->derivativeValue) + (1.0f - PID->alpha)*(processValue - PID->previousProcessValue)/(PID->dt);
  
  double controlValue = PID->Kp * Proportional + PID->Ki * Integral + PID->Kd * Derivative;

  
  //Set Limits
  if(controlValue > 1){
      controlValue = 1;
      
      if(errorValue < 0){ //Prevent Integral Windup
        PID->integralValue = PID->Ki * Integral;
      }
  } else if(controlValue < 0){
      controlValue = 0;

      if(errorValue > 0){ //Prevent Integral Windup
        PID->integralValue = PID->Ki * Integral;
      }
  } else {
      
      PID->integralValue = PID->Ki * Integral; // Normal Operation
  }
  
  //Update Parameters
  PID->derivativeValue = PID->Kd * Derivative;
  PID->previousProcessValue = processValue;

  return controlValue;
}

void PID_RESET(struct PID_parameters *PID){
  PID->integralValue = 0;
  PID->previousProcessValue = 0;
  PID->derivativeValue = 0;
}
