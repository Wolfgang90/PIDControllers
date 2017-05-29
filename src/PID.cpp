#include <math.h>
#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd){

  // Parameters
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Errors
  p_error = 0;
  i_error = 0;
  d_error = 0;


  iteration_counter = 0;
}

void PID::UpdateError(double cte) {
  
  // Update errors
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte; 

  iteration_counter += 1;
}



double PID::TotalError() {
  double total_error = -p_error * Kp - i_error * Ki - d_error * Kd;

  total_error = Sigmoid(total_error,-1.0,1.0);  

  return total_error;
}



double PID::Sigmoid(double input, double low, double high){
  double gap = high - low;
  return gap / ( 1 + exp(-input)) + low;
}
