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
   * Number of updates in a lap.
   */
  int cnt_max;

  /*
   * Sum of squre error.
   */
  double lap_err;
  double prev_lap_err;
  double best_lap_err;

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
  double prev_p_error;

  /*
  * Coefficients
  */ 
  std::vector<double> coeff;
  std::vector<double> dCoeff;
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

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
