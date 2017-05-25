#include "helper.h"
#include <math.h>

double Sigmoid(double input, double low, double high){
  double gap = high - low;
  return gap / ( 1 + exp(-input)) + low;
}
