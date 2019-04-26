#include "filter.h"
#include <iostream>
#include <math.h>       /* fabs */

using namespace std;


filter::filter() {}

filter::~filter() {}

void filter::Init(double alpha_) {

    cout<<"Initializing Filter \n";

    alpha = alpha_;                       // Proportionnal term
    prev_value = 0;
  }

void filter::savePrevious(double value) {

      prev_value = value;
  }


double filter::smooth(double value) {
  return alpha*value+(1-alpha)*prev_value;
}