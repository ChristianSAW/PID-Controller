#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  prev_cte = 0;
  sum_cte = 0;
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  stp = 0; 
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  sum_cte += cte;
  p_error = -cte*Kp;
  d_error = -(cte-prev_cte)*Kd;
  i_error = -sum_cte*Ki;
  prev_cte = cte; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error + d_error + i_error;  // TODO: Add your total error calc here!
}

double PID::update_steer(double cte) {
  ++stp;
  UpdateError(cte);
  return TotalError();
}

double PID::get_stp() {
  return stp;
}