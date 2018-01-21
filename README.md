# PID Control Project

A proportional-integral-derivative (PID) controller is implemented to control a car around a track.
The controller works with a simulator that provides the cross track error (CTE) and the velocity to calculate steering angle.
The communication between the controller and the simulator is done through [uWebSockets](https://github.com/uWebSockets/uWebSockets)

---

## Steering Control

The simulator continuously calculates the CTE of a vehicle on the track. A PID controller applies correction according to the proportional, integral, and derivative of the observed CTE. Three parameters control the magnitude of the correction, and they are named Kp, Ki, and Kd. The calculation is implemented in function `PID::Steering()`.

```c++
  double controlP = - K[0] * pid_err[0];
  double controlI = - K[1] * pid_err[1];
  double controlD = - K[2] * pid_err[2];

  // Reduce steering at high speed.
  double speed_factor = (current_speed > 40) ? 40./current_speed : 1.;
  double control = (controlP + controlI + controlD) * speed_factor;
  return control;
```

`K` is a vector three elements, Kp, Ki, and Kd. `pid_err` is a vector that holds the proportional, integral, and derivative errors. I find thatt it helps to reduce the steering in high speeds, so I added `speed_factor` to control the steering value at high speeds.

## Throttle Control

I modified the original code and increased the throttle value from 0.3 to 2.0 to make the control more challenging. It was harder to control the vehicle at higher speed. One way to take the control back was to reduce the speed when the errors are larger. The function `PID::Throttle()` includes a variable `speed_factor` that reduces the value of throttle at the speed higher than 40. The reduction is proportaional to the total error and the `speed_factor`.

## Parameter Tuning

The parameters Kp, Ki, and Kd are manually set in following steps.

1. Set Kp, Ki, and Kd to zeros.
2. Increase Kp until the vehicle passes the first curve without going off the track.
3. Increase Kd until the oscillation of the CTE is minimized.
4. Increase Ki up to the maximum that doesn't seem to introduce destabilization.
5. If the vehicle doesn't pass the steeper curve, repeat the steps 2-4.

Once the vehicle completes the lap with the parameters that are manually set, I run the twiddle algorithm to further optimze the parameters.
The twiddle algorithm is implemented in the function `PID::Twiddle()`.

I assumed that 1300 calculations are performed in each lap, which is an approximation. Following table shows how the error improves during 19 laps.

| Lap | Kp        | Ki       |       Kd | Best Error(lap) | Error(lap) |
|:---:|:---------:|:--------:|:--------:|:----------:|:---------------:|
| 1   | 0.10000000|0.00200000|1.50000000| N/A        | 786.82          |
| 2   | 0.10100000|0.00200000|1.50000000| 786.82     | 1109.37         |
| 3   | 0.09900000|0.00200000|1.50000000| 786.82     | 763.70          |
| 4   | 0.09900000|0.00202000|1.50000000| 763.70     | 1127.40         |
| 5   | 0.09900000|0.00198000|1.50000000| 763.70     | 757.02          |
| 6   | 0.09900000|0.00198000|1.51500000| 757.02     | 1105.46         |
| 7   | 0.09900000|0.00198000|1.48500000| 757.02     | 767.59          |
| 8   | 0.10010000|0.00198000|1.50000000| 757.02     | 1098.70         |
| 9   | 0.09790000|0.00198000|1.50000000| 757.02     | 839.41          |
| 10  | 0.09900000|0.00200200|1.50000000| 757.02     | 970.59          |
| 11  | 0.09900000|0.00195800|1.50000000| 757.02     | 906.82          |
| 12  | 0.09900000|0.00198000|1.51350000| 757.02     | 978.29          |
| 13  | 0.09900000|0.00198000|1.48650000| 757.02     | 902.34          |
| 14  | 0.09999000|0.00198000|1.50000000| 757.02     | 945.50          |
| 15  | 0.09801000|0.00198000|1.50000000| 757.02     | 948.16          |
| 16  | 0.09900000|0.00199980|1.50000000| 757.02     | 857.64          |
| 17  | 0.09900000|0.00196020|1.50000000| 757.02     | 988.49          |
| 18  | 0.09900000|0.00198000|1.51215000| 757.02     | 859.85          |
| 19  | 0.09900000|0.00198000|1.48785000| 757.02     | 1054.27         |

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 







































## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

