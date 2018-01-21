#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
   * Run twiddle optimization if true.
   */
  bool do_optimize;
  
  /*
   * Number of updates per lap.
   */
  int cnt_max;

  /*
   * Sum of squre error during lap.
   */
  double lap_err;

  /*
   * Sum of squre error from the best lap.
   */
  double best_lap_err;

  /*
   * Current speed.
   */
  double current_speed;

  /*
   * Current angle.
   */
  double current_angle;

  /*
   * Keep track of twiddle process.
   */
  enum TwiddleState {
    undefined, added, subtracted
  };

  /*
  * Errors
  */
  std::vector<double> pid_err;
  double prev_p_err;

  /*
  * Coefficients
  */ 
  std::vector<double> K;
  std::vector<double> dK;
  int n_err;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed, double angle);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Get steering value.
  */
  double Steering();

  /*
  * Get throttle value.
  */
  double Throttle();

  /*
   * Turn on optimization.
   */
  void DoOptimize();

  /*
  * Optimize coefficients.
  */
  void Twiddle();
};

#endif /* PID_H */
