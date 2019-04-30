#include "PID.h"
#define WINDOW_SIZE 20

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

  // For 3rd PID Approach (Use of Sliding Window for cte sum)
  window = (int*) calloc( WINDOW_SIZE , sizeof(window[0]) ) ;
  i = WINDOW_SIZE - 1 ;
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

void PID::UpdateError2(double cte, double dt) {
  p_error = -cte;
  d_error = -(cte-prev_cte);
  i_error = -add_i(cte * dt);
  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error + d_error + i_error;  // TODO: Add your total error calc here!
}

double PID::update_val(double cte) {
  ++stp;
  UpdateError(cte);
  return TotalError();
}

double PID::update_steering_lin(double cte, double speed, dt) {
  double Kp_sp = 0.0032;
  double Kd_sp = 0.0002;
  ++stp;
  UpdateError2(cte,dt);
  return (Kp - Kp_sp*speed)*p_error + (Kd + Kd_sp*speed)*d_error + Ki*i_error
}

int PID::get_stp() {
  return stp;
}

// Taken from Nickolas Ent, https://github.com/NikolasEnt/PID-controller/blob/master/src/PID.cpp
// For i_error calculation we sum up only WINDOW_SIZE measurements
// inspired by code from https://stackoverflow.com/questions/25024012/running-sum-of-the-last-n-integers-in-an-array
double PID::add_i(double err){
  i = (i+1) % WINDOW_SIZE;
  sum_cte = sum_cte - window[i] + err ;
  return sum_cte;
}
