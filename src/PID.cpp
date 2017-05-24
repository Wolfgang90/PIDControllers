#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd): Kp(Kp), Ki(Ki), Kd(Kd), p_error(0), i_error(0), d_error(0){
}

PID::~PID() {}


void PID::UpdateError(double cte) {
  
  // Initialize last measure (only required during first call of update error)
  if(!t_last_meas){
    t_last_meas = std::chrone::steady_clock::now();
    }

  // Calculate interval since last measurement
  auto t_curr = std::chrone::steady_clock::now();
  std::chrono::duration<double> interval = t_curr - t_last_meas;
  double dt_interval = interval.count();
  t_last_meas = t_curr;

  // Update errors
  d_error = (cte - p_error) * dt_interval;
  p_error = cte;
  i_error += cte * dt_interval; 
}

double PID::TotalError() {
  return -p_error * Kp - i_error * Ki - d_error * Kd;
}

