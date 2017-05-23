#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd): Kp(Kp), Ki(Ki), Kd(Kd), p_error(0), i_error(0), d_error(0){
}

PID::~PID() {}


void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

