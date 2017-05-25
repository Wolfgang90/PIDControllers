#include <math.h>
#include <iostream>
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
  d_error = (cte - p_error) / dt_interval;
  p_error = cte;
  i_error += cte * dt_interval; 

  iteration_counter += 1;

  if(iteration_counter > twiddle_threshold){
    ApplyTwiddle(cte, twiddle_tolerance);
  }
}



double PID::TotalError() {
  double total_error = -p_error * Kp - i_error * Ki - d_error * Kd;
  
  return total_error;
}




void PID::ApplyTwiddle(double cte, double tolerance){
   std::cout << "Parameters before twiddle:" << std::endl;
  std::cout << "Kp -> " << Kp << "; Ki -> " << Ki << "; Ki -> " << Kd << std::endl;
 
  std::vector<double> parameters = {Kp, Ki, Kd};
  std::vector<double> dp = {1.0, 1.0, 1.0};

  double best_err = TotalError();


  std::cout << "Initial best_err: " << best_err << endl;


  while(std::accumulate(dp.begin(), dp.end(), 0) > tolerance){
    double err;
    PID twiddle;

    for(int i = 0; i < dp.size(); i++){
      parameters[i] += dp[i];

      twiddle.Init(parameters[0], parameters[1], parameters[2]);
      twiddle.p_error = p_error;
      twiddle.i_error = i_error;
      twiddle.d_error = d_error;


      std::cout << "Twiddle instance p_error: " << twiddle.p_error << std::endl;
      std::cout << "p_error: " << p_error << std::endl;


      twiddle.UpdateError(cte);

      err = twiddle.TotalError();

      if(err < best_err){
        best_err = err;
        dp[i] *= 1.1;
      } else{
        parameters[i] -= 2 * dp[i];
        twiddle.Init(parameters[0], parameters[1], parameters[2]);
        twiddle.p_error = p_error;
        twiddle.i_error = i_error;
        twiddle.d_error = d_error;

        twiddle.UpdateError(cte);

        err = twiddle.TotalError();

        if(err < best_err){
          best_err = err;
          dp[i] *= 1.1;
        } else{
          parameters[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }


  std::cout << "Final best error: " << best_err << endl;


  Kp = parameters[0];
  Ki = parameters[1];
  Kd = parameters[2];
  std::cout << "New parameters after twiddle:" << std::endl;
  std::cout << "Kp -> " << Kp << "; Ki -> " << Ki << "; Ki -> " << Kd << std::endl;
  exit(EXIT_FAILURE);
}


double PID::Sigmoid(double input, double low, double high){
  double gap = high - low;
  return gap / ( 1 + exp(-input)) + low;
}
double Sigmoid(double input, double low, double high){
  double gap = high - low;
  return gap / ( 1 + exp(-input)) + low;
}

