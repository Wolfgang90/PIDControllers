#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>
#include <numeric>

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
  * Iteration counter
  */
  int iteration_counter;

  /*
  * Initializing PID
  */
  void Init(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  double Sigmoid(double input, double low, double high);
};

#endif /* PID_H */
