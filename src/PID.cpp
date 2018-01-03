#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
  :pid_err({0., 0., 0.}), // P, I, D
  n_err(3),
  prev_p_error(0.),
  lap_err(0.),
  prev_lap_err(0.),
  best_lap_err(0.),
  do_optimize(false),
  cnt_max(1700)
{
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  coeff = {Kp, Ki, Kd};
  
  // Twiddle parameters.
  for(auto c : coeff)
    dCoeff.push_back(.01 * c);
}

void PID::UpdateError(double cte) {
  pid_err[0] = cte;
  pid_err[1] = cte - prev_p_error;
  prev_p_error = cte;
  pid_err[2] += cte;
}

double PID::TotalError() {
  if(do_optimize)
    Twiddle();

  return -coeff[0]*pid_err[0] - coeff[1]*pid_err[1] - coeff[2]*pid_err[2];
}

void PID::DoOptimize() {
  do_optimize = true;
}

void PID::Twiddle() {
  static int cnt = 0;
  static int iCoeff = 0;
  static TwiddleState state = added;

  // Sum of the CTE during lap
  lap_err += pid_err[0] * pid_err[0];

  // Run twiddle after each lap.
  ++cnt;
  if(cnt >= cnt_max) {
    printf("C %.8f %.8f %.8f err %.2f %.2f\n", coeff[0], coeff[1], coeff[2], best_lap_err, lap_err);
    cout.flush();

    if(best_lap_err == 0.) {
      best_lap_err = lap_err;
      coeff[iCoeff] += dCoeff[iCoeff];
      state = added;
    }
    else {
      if(state == added) {
        if(lap_err < best_lap_err) {
          best_lap_err = lap_err;
          dCoeff[iCoeff] *= 1.1;
          iCoeff = (iCoeff + 1) % n_err;
          coeff[iCoeff] += dCoeff[iCoeff];
          state = added;
        }
        else {
          coeff[iCoeff] -= 2.*dCoeff[iCoeff];
          state = subtracted;
        }
      }
      else if(state == subtracted) {
        if(lap_err < best_lap_err) {
          best_lap_err = lap_err;
          dCoeff[iCoeff] *= 1.1;
          iCoeff = (iCoeff + 1) % n_err;
          coeff[iCoeff] += dCoeff[iCoeff];
          state = added;
        }
        else {
          coeff[iCoeff] += dCoeff[iCoeff];
          dCoeff[iCoeff] *= 0.9;
          iCoeff = (iCoeff + 1) % n_err;
          coeff[iCoeff] += dCoeff[iCoeff];
          state = added;
        }
      }
    }
    cnt = 0;
    lap_err = 0.;
  }
}
