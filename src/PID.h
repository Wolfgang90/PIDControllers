#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>
#include <numeric>
#include "helper.h"

class PID {
  std::chrono::steady_clock::time_point t_last_meas;

public:

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  /*
  * Number of epochs after which twiddle optimization should be applied
  */
  int twiddle_threshold;

  /*
  * Twiddle tolerance value for accumulated dp-values
  */
  double twiddle_tolerance;

  /*
  * Iteration counter
  */
  int iteration_counter;

  /*
  * Initializing PID
  */
  void Init(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, int twiddle_threshold = 100, double twiddle_tolerance = 0.0001);

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
  void ApplyTwiddle(double cte, double tolerance);

  double Sigmoid(double input, double low, double high);

  /*
  * Vector for changing factors for P, I and D
  */
  std::vector<double> dp {1.0,1.0,1.0};

};

#endif /* PID_H */
