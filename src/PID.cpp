#include "PID.h"
#include "helper.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int twiddle_threshold, double twiddle_tolerance){

  // Parameters
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Errors
  p_error = 0;
  i_error = 0;
  d_error = 0;

  // Number of epochs after which twiddle optimization should be applied
  this->twiddle_threshold = twiddle_threshold;

  // Twiddle tolerance value for accumulated dp-values
  this->twiddle_tolerance = twiddle_tolerance;


  iteration_counter = 0;

  t_last_meas = std::chrono::steady_clock::now();
}

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

  if(iteration_counter < twiddle_threshold){
    ApplyTwiddle(twiddle_tolerance);
  }
}



double PID::TotalError() {
  double total_error = -p_error * Kp - i_error * Ki - d_error * Kd;
  
  total_error = Sigmoid(total_error);
  return total_error;
}




void PID::ApplyTwiddle(double tolerance){
  
  std::vector<double> parameters = {Kp, Ki, Kd};
  std::vector<double> dp = {1.0, 1.0, 1.0};

  double best_err = TotalError();

  while(std::accumulate(dp.begin(), dp.end(), 0) > tolerance){
    PID twiddle;

    for(int i = 0; i < dp.size(); i++){
      parameters[i] += dp[i];
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
