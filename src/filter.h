
#ifndef FIL_H
#define FIL_H


class filter {
public:
  /*
  * Coefficients
  */
  double alpha;
  double prev_value;

  /*
  * Constructor
  */
  filter();

  /*
  * Destructor.
  */
  virtual ~filter();

  /*
  * Initialize porportional term alpha.
  */
  void Init(double alpha);

  /*
  * Save prior value.
  */
  void savePrevious(double value);

  /*
  * Calculate new smoothed value.
  */
  double smooth(double value);
};


#endif /* FIL_H */