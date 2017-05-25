#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd, int twiddle_threshold): Kp(Kp), Ki(Ki), Kd(Kd), twiddle_threshold(twiddle_threshold), p_error(0), i_error(0), d_error(0),t_last_meas(std::chrono::steady_clock::now()){
}

PID::~PID() {}


void PID::UpdateError(double cte) {
  
  // Calculate interval since last measurement
  auto t_curr = std::chrono::steady_clock::now();
  std::chrono::duration<double> interval = t_curr - t_last_meas;
  double dt_interval = interval.count();
  t_last_meas = t_curr;

  // Update errors
  d_error = (cte - p_error) * dt_interval;
  p_error = cte;
  i_error += cte * dt_interval; 

  iteration_counter += 1;

  ApplyTwiddle(0.2);
}

double PID::TotalError() {
  return -p_error * Kp - i_error * Ki - d_error * Kd;
}

void PID::ApplyTwiddle(double tolerance){
  if(iteration_counter < twiddle_threshold){
    return;
  }

  double best_err = PID::TotalError();

  while(std::accumulate(dp.begin(), dp.end(), 0) > tolerance){
    for(int i = 0; i < dp.size(); i++){
      switch(i){
        case 0:
          Kp += dp[i];
          break;
        case 1:        
          Ki += dp[i];
          break;
        case 2:
          Kd += dp[i];
          break;
      }
      
      double err = PID::TotalError();

      if(err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      } else{
        switch(i){
          case 0:
            Kp -= 2 * dp[i];
            break;
          case 1:        
            Ki -= 2 * dp[i];
            break;
          case 2:
            Kd -= 2 * dp[i];
            break;
        } 
        err = PID::TotalError();

        if(err < best_err){
          best_err = err;
          dp[i] *= 1.1;
        } else{
          switch(i){
            case 0:
              Kp += dp[i];
              break;
            case 1:        
              Ki += dp[i];
              break;
            case 2:
              Kd += dp[i];
              break;
          }
          dp[i] *= 0.9;
        }
      }
    }
  }
}
