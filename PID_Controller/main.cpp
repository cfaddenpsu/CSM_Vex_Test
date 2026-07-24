#include <iostream>
#include "PID.h"

//Valeria Was Here


int main(int argc, char **argv){
  
  /***********************
   * Setup PID Controller
   **********************/
  //Create PID Controller 
  struct PID_parameters* Quad_PID = new struct PID_parameters;
  
  //Set Coefficient Values
  Quad_PID->Kp = 0.3;
  Quad_PID->Ki = 0.5;
  Quad_PID->Kd = 0.01;
    
  
  //Objective function is (x - 0.5)^2 = x*x - x + 0.25;  Minimum at x = 0.5

  //Setpoint
  double set_value = 0.5; 
  
  //Initial Value
  double processValue = 0.25;

  //Create PID Loop
  PID_RESET(Quad_PID);
  for(int ii = 0; ii < 40; ii++){
    
    if( ii % 5 == 0){
      std::cout << "The process value (should be 0.5) is: " << processValue << std::endl;
    }
    processValue = processValue + PID_update(set_value, processValue, Quad_PID);
    
  }
  
  return 0;
}
