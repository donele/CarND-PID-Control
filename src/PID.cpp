#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
  :pid_err({0., 0., 0.}), // P, I, D
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
  K = {Kp, Ki, Kd};
  
  // Twiddle parameters.
  for(auto k : K)
    dK.push_back(.01 * k);
}

void PID::UpdateError(double cte, double speed, double angle) {
  pid_err[0] = cte;

  if(cte < 1e-6)
    pid_err[1] = 0.;
  else
    pid_err[1] += cte;

  pid_err[2] = cte - prev_p_err;
  prev_p_err = cte;

  current_speed = speed;
  current_angle = angle;

  if(do_optimize)
    Twiddle();
}

double PID::TotalError() {
}

double PID::Steering() {
  double controlP = - K[0] * pid_err[0];
  double controlI = - K[1] * pid_err[1];
  double controlD = - K[2] * pid_err[2];

  // Reduce steering at high speed.
  double speed_factor = (current_speed > 40) ? 40./current_speed : 1.;
  double control = (controlP + controlI + controlD) * speed_factor;
  return control;
}

double PID::Throttle() {
  double controlP = K[0] * pid_err[0];
  double controlI = K[1] * pid_err[1];
  double controlD = K[2] * pid_err[2];

  // Reduce throttle above speed = 40.
  // The amount of reduction is proportinal to the error.
  const double throttle_max = 2.;
  double speed_factor = (current_speed > 40) ? current_speed/5. : 0.;
  double throttle = throttle_max - fabs(controlP + controlI + controlD) * speed_factor;
  return throttle;
}

void PID::DoOptimize() {
  do_optimize = true;
}

void PID::Twiddle() {
  static int cnt = 0;
  static int iK = 0;
  static TwiddleState state = added;

  // Sum of the CTE during lap
  lap_err += pid_err[0] * pid_err[0];

  // Run twiddle after each lap.
  ++cnt;
  if(cnt >= cnt_max) {
    printf("C %.8f %.8f %.8f err %.2f %.2f\n", K[0], K[1], K[2], best_lap_err, lap_err);
    cout.flush();

    // First lap. Update the best error and proceed with twiddle.
    if(best_lap_err == 0.) {
      best_lap_err = lap_err;
      K[iK] += dK[iK];
      state = added;
    }
    else {
      // If dK[iK] was added to K[iK] at the beginning of the current lap.
      if(state == added) {
        // If lap_err is smaller than the previous best, update the best error.
        if(lap_err < best_lap_err) {
          best_lap_err = lap_err;
          dK[iK] *= 1.1;
          iK = (iK + 1) % n_err;
          K[iK] += dK[iK];
          state = added;
        }
        // If lap_err is larger than the previous best, subtract dK[iK] from K[iK].
        else {
          K[iK] -= 2.*dK[iK];
          state = subtracted;
        }
      }
      // If dK[iK] was subtracted from K[iK] at the beginning of the current lap.
      else if(state == subtracted) {
        // If lap_err is smaller than the previous best, update the best error.
        if(lap_err < best_lap_err) {
          best_lap_err = lap_err;
          dK[iK] *= 1.1;
          iK = (iK + 1) % n_err;
          K[iK] += dK[iK];
          state = added;
        }
        // If lap_err is larger than the previous best, reduce dK[iK].
        else {
          K[iK] += dK[iK];
          dK[iK] *= 0.9;
          iK = (iK + 1) % n_err;
          K[iK] += dK[iK];
          state = added;
        }
      }
    }
    cnt = 0;
    lap_err = 0.;
  }
}
