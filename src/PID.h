#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);
  
   /**
   * Update the PID error variables given cross track error and dt.
   * @param cte The current cross track error
   * @param dt The current time delta for this time stamp 
   */
  void UpdateError2(double cte, double dt);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Update the steering (or other) value given cross track (or other) error.
   * @param cte The current cross track error (o)r other error)
   */
  double update_val(double cte);

  /**
   * Update the steering value given cte, speed, and dt.
   * @param cte The current cross track error
   * @param speed The current speed of the car
   * @param dt The current time delta for this time stamp 
   */
  double update_steering_lin(double cte, double speed, double dt);
  
  /**
   * Returns the current step (counter)
   */
  int get_stp();

  /**
   * Returns sliding window addition of err
   * @param err Error to be cumulatively added
   */
  double add_i(double err);

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double sum_cte;


 private:
  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /**
   * Sliding Window Addition
   */
  int *window;
  int i;

  /**
   * Parameter Optimization
   */
  int stp = 0; // counter

};

#endif  // PID_H
