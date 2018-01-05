#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
  :pid_err({0., 0., 0.}), // P, I, D
  coeff_throttle(15),
  n_err(3),
  prev_p_err(0.),
  lap_err(0.),
  best_lap_err(0.),
  do_optimize(false),
  cnt_max(1300)
{
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  coeff = {Kp, Ki, Kd};
  
  // Twiddle parameters.
  for(auto c : coeff)
    dCoeff.push_back(.01 * c);
}

void PID::UpdateError(double cte, double speed, double angle) {
  pid_err[0] = cte;
  pid_err[1] += cte;
  pid_err[2] = cte - prev_p_err;
  prev_p_err = cte;

  current_speed = speed;
  current_angle = angle;
}

double PID::TotalError() {
}

double PID::Steering() {
  if(do_optimize)
    Twiddle();

  double controlP = 0.;
  double controlI = 0.;
  double controlD = 0.;

  // Reduce steering at high speed.
  double speed_factor = (current_speed > 40) ? 40./current_speed : 1.;

  controlP = - coeff[0]*pid_err[0]*speed_factor;
  controlI = - coeff[1]*pid_err[1]*speed_factor;
  controlD = - coeff[2]*pid_err[2]*speed_factor;
  return controlP + controlI + controlD;
}

double PID::Throttle() {
  // Reduce throttle above speed = 40.
  // The amount of reduction is proportinal to the error.
  double speed_factor = (current_speed > 40) ? current_speed/100. : 0.;

  double controlP = coeff[0]*pid_err[0];
  double controlI = coeff[1]*pid_err[1];
  double controlD = coeff[2]*pid_err[2];
  double throttle = 2. - coeff_throttle * fabs(controlP + controlI + controlD)*speed_factor;
  return throttle;
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
