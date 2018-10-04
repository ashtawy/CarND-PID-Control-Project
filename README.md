# PID Controller

## Background
The goal of this project is to control a vehicle driving on a road by keeping it as close to the center of the road as possible. We utilize the PID controller to achieve this goal. We use a simulator of a car driving on a road and real-time stream of its speed, cross-track error (CTE), and steering angle. The simulator allows the user to control the car using its throttle and steering angle values, which will be provided by our PID controllers.

The steering angle PID controller we use here attempts to minimize the error over time by adjusting the steering angle of the car, such as the location of the car, to a new value determined by a weighted sum of the control terms. The control terms of the PID controller are:

*Steering angle = -P\*CTE - I\*CTE<sub>acc</sub> - D\*(CTE<sub>t</sub>-CTE<sub>t-1</sub>)*

* *P* is the proportional term to the error between the desired and actual location of the car (CTE). The higher the error, the larger the contribution of this term to the control signal.
* *I* helps eliminate any systematic drift the car may have from the desired location. The *I* parameter is multiplied by the accumulated CTE (over time) to remove the drift effect.
* *D* accounts for the rate of change of the error CTE. The higher the rate of change of error *(CTE<sub>t</sub>-CTE<sub>t-1</sub>)*, the higher the damping effect.

In addition to the steering angle PID controller, we also use a throttle PID controller. It uses the magnitude of error (i.e., the abs value) to compute the control signal.

## Parameter Selection
The challenge in this project is to carefully tune the parameters of the PID controllers (P, I, & D) such that the generated steering angle and throttle control signals keep the car inside the road *all* the time. 

In this project, the controller parameters were initially manually selected and then automatically tuned using the *twiddle* algorithm to further decrease the overall CTE. The manual selection step through trial and error is necessary to make sure that the car is driving on the road and not trapped outside when we fine tune the parameters in the automated mode. Also, during the course of fine tuning, the proposed changes to the parameters should be small enough to ensure that the car continues driving on the road and large enough to quickly find the optimal values. The twiddle algorithm runs for 1000 steps of driving to test whether the proposed change is acceptable or not. If acceptable, the next change will be slightly larger (10% of the current change). Otherwise, we will reduce the change by 10% and try again.

After compiling the project (as outlined in the steps below), the project can be ran in the driving mode or driving & twiddling mode using the commands:
```
driving_mode_only$./pid 
diving_and_twiddling_mode$./pid twiddle
```
  


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
4. Run it:
    * in driving mode only: `./pid`.
    * in driving and parameter tunning modes (twiddle): `./pid twiddle` 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

The editor configuration files were kept out of this repo in order to
keep it as simple and environment agnostic as possible. However, it is recommended to use the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).



