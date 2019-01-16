# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# Model
We are using Kinematic Model which ignores forces, gravity and mass. In this model we just keep track of State of the vehicle.
### Properties in the State
- X position
- Y Position
- Orientation
- Velocity

The aim of the model is to predict the future state over time given the current state and actuator inputs. The actuator inputs allows to control the vehicle state.

- Accelerator
- Throttle (Here both acceleration(Positive value) and braking(Negative value))

State: [x,y,ψ,v]</br>
Actuators: [δ,a]</br>
![Equations](https://github.com/VenkatRepaka/CarND-MPC-Project/blob/master/notes/state_equations.JPG)


Now the reference trajectory is derived from a polynomial using 3rd order, since third order polynomials will fit the trajectories for most roads. This is done using Eigen library in C++
### Minimize Error
The error between the reference trajectory and the actual trajectory should be minimized.
This can be done by predicting the vehicle's actual path and then adjusting the control inputs to minimize the difference between that prediction and reference trajectory.
##### Cross Track Error
![Equations](https://github.com/VenkatRepaka/CarND-MPC-Project/blob/master/notes/cte_equation.JPG)</br>
f(x) is our reference line. It is the position of y given x in the polynomial.
##### Orientation Error
![Equations](https://github.com/VenkatRepaka/CarND-MPC-Project/blob/master/notes/orientation_equation.JPG)</br>
![Equations](https://github.com/VenkatRepaka/CarND-MPC-Project/blob/master/notes/orientation_equation_explanation.JPG)</br>
ψdes can be calculated as tangential angle of the polynomial f evaluated at x.

##### Actuator Constraints
Actuators are limited by design and fundamental physics. Below are the upper and lower bounds that have been used in the project.
- The upper and lower bounds for steering angle are 25deg and -25deg.
- The upper and lower bounds for throttle angle are 1 and -1. 1 means full acceleration and -1 means full brake.


# MPC
### Cost Reduction
- Minimize error to actual state(cte, epsi and velocity)
- Minimize the use of actuators.
- Minimize the value gap between sequential actuations.

### Length and Duration
_N_ is the number of timesteps in the horizon. _dt_ is how much time elapses between actuations. The values chosen for _N_ is 10 and for _dt_ is 0.1

## Solution

The waypoints have to be transformed into points of vehicle coordinate system.
```
Eigen::VectorXd car_x(ptsx.size());
Eigen::VectorXd car_y(ptsy.size());
double x_diff, y_diff, x_car, y_car;
double minus_psi =  -1 * psi;
for(unsigned  int i=0;i<ptsx.size();i++) {
	x_diff = ptsx[i] - px;
	y_diff = ptsy[i] - py;
	x_car = x_diff *  cos(minus_psi) - y_diff *  sin(minus_psi);
	y_car = x_diff *  sin(minus_psi) + y_diff *  cos(minus_psi);
	car_x[i] = x_car;
	car_y[i] = y_car;
}
```
The cost function parameters were tuned using trial and error method and the final values are 
```
fg[0] +=  1000*CppAD::pow(vars[cte_start + i], 2);
fg[0] +=  1000*CppAD::pow(vars[epsi_start + i], 2);
fg[0] +=  CppAD::pow(vars[v_start + i] - ref_v, 2);

// Minimize the use of actuators.
fg[0] += 50  *  CppAD::pow(vars[delta_start + i], 2);
fg[0] += 50  *  CppAD::pow(vars[a_start + i], 2);
// including error for speed and angle
// Very good at higher reference speeds but slowing more than required at low speeds. Unable to gain max speed.
// But when used at lower ref speeds this lowered the speed at turning more than required.
// This helped most at high speeds. Slowed at turnings which helped avoid car over turns/off road experiences
if(ref_v >  50) {
	fg[0] +=  200  *  CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
}
else  if(ref_v >  100) {
	fg[0] +=  500  *  CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
}
else  if(ref_v >  150) {
	fg[0] +=  800  *  CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
}

// Minimize the value gap between sequential actuations.
if(ref_v <  100) {
	fg[0] +=  25000  *  CppAD::pow(vars[delta_start + i +  1] - vars[delta_start + i], 2);
}
else  if(ref_v <  150) {
	fg[0] +=  100000  *  CppAD::pow(vars[delta_start + i +  1] - vars[delta_start + i], 2);
}
else {
	fg[0] +=  250000  *  CppAD::pow(vars[delta_start + i +  1] - vars[delta_start + i], 2);
}
if(ref_v <  150) {
	fg[0] +=  5000  *  CppAD::pow(vars[a_start + i +  1] - vars[a_start + i], 2);
}
else {
	fg[0] +=  100000  *  CppAD::pow(vars[a_start + i +  1] - vars[a_start + i], 2);
}
```

We are running the vehicle controls with a latency. dt used is 100 milliseconds. For this the relative state for 100 milliseconds has to be calculated and this is used as the initial state for MPC.
```
// Since we measure from car position both x and y are 0.
const double dt = 0.1;
const double Lf = 2.67;
// psi = 0;
// cos(0) is 1
// px = v * cos(psi) * dt;
px = v * dt;
// sin(0) = 0
// py = v * sin(psi) * dt;
py = 0;
psi = -1 * v * delta * dt / Lf;
v += a * dt;
cte = 0;
epsi -= psi;
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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
