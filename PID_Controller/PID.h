#pragma once

struct PID_parameters {
  double Kp = 0;  //Proportional Gain Constant
  double Ki = 0;  //Integral Gain Constant 
  double Kd = 0;  //Derivative Gain Constant
  
  double dt = 0.01;  //Time Step in [s]
  double integralValue = 0; //Accumulated Integral Value
  double previousProcessValue = 0; 
  double derivativeValue = 0; 


  double alpha = 0.91; //Derivative Filter Constant in [0, 1]; (Higher = More Smoothing)  
};

//Reset all loop PID variables to default state
void PID_RESET(struct PID_parameters *PID);

//Run a single iteration of a PID loop
double PID_update(double setValue, double processValue, struct PID_parameters *PID);


