#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd): Kp(Kp), Ki(Ki), Kd(Kd), p_error(0), i_error(0), d_error(0){
}

PID::~PID() {}


void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte; 
}

double PID::TotalError() {
  return -p_error * Kp - i_error * Ki - d_error * Kd;
}

