#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>
#include <numeric>

class PID {
  std::chrono::steady_clock::time_point t_last_meas;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, int twiddle_threshold = 100);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Apply twiddle to find optimal parameters
  */
  void ApplyTwiddle(double tolerance);

  /*
  * Vector for changing factors for P, I and D
  */
  std::vector<double> dp {1.0,1.0,1.0};

  /*
  * Iteration counter
  */
  int iteration_counter = 0;
  
  /*
  * Minimum number of iterations before to apply twiddle
  */

  int twiddle_threshold;


};

#endif /* PID_H */
