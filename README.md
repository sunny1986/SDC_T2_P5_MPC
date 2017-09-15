# Project 5 Model Predictive Control


 In this project we use Model Predictive Control to drive a car around a track in a simulator. In this approach we use Kinematic model of the car. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.

 ## THE MODEL:

 A vehicle in map co-ordinates has two states which represent the x & y locations in this co-ordinate system. Also the orienation of the vehicle wrt to the x axis in this system is a part of the state of the vehicle given by psi. Since the vehicle is moving and has finite velocity in a particular direction, we use the velocity v as one of that states too. So till now our state is described by [x, y, psi, v]. 

 One part of the project is to derive a model that captures how the state evolves over time and how the inputs interact with the states and influence the future state of the vehicle. Inputs influence the state and in our case we have two actuators, the steering wheel angle and the accelerator as inputs. In case of accelerator +ve value means acceleration and -ve values means the vehicle is breaking. Hence we use 2 inputs, delta for steering angle and a for acceleration.

 In our case, we are getting the way points and the state of the vehicle from the simulator (lines 88 to 94). Using these waypoints and the current state, we have to calculate the control inputs. This is where the MPC controller is used.

 First we convert the current steering angle received from the simulator into radians and transform the waypoints into car co-ordinate system to work out all calculations with a single reference frame. A subsequet 2D rotation to align the x-axis with the heading direction. Therby the waypoints are obtained in the frame of the vehicle. FOllowing are the model equations
      
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

 ## CHOOSING N & dt:

 We choose the time interval over which we plan to predict the vehicle trajectory. This time is T = N * dt. The shorter the prediction time T, the faster the calculation of the predicted trajectory but it might lead to instability since the correction is happening too quickly. The longer the T is, the longer it will take to calcuate but also it may be less accuracte since there might be other parameters that did not get covered in the calculations for that long time interval. Hence, T having a value somewhere in between where the controller is also stable but smooth is chosen. In general, smaller dt gives better accuracy but that will require higher N for given horizon (N*dt). However, increase N will result in longer computational time which effectively increase the latency  Admittedly, I used the values of N and dt to be 10 and 0.1 sec from the suggestions in the Q&A video and the forum posts. This turns out to predicting the trajectory of the car over 10 * 0.1 = 1 sec into the future.  

 Next we use the waypoints and use polyfit to fit a 3rd order polynomial to those waypoints and get the coeffs of the polynomial.  These coeffs are used in calculating (a) the cross track error (cte): the error between the desired trajectory and the actual trajectory of the vehicle, (b) the orientation error (epsi): orientation of the vehicle wrt the direction of desired trajectory. 

 ## LATENCY:

 In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. This is a problem called LATENCY. The car drifts at the current speed, heading, and rate of turn for the entire interval forward. Latency happens due to the actuator dynamics. For example, there is a delay between when the steering angle was commanded to go to a certain value and when it actually reaches that value. One advantage of the MPC controller is that we can model this latency into the vehicle model. Lines 142-148 takes care of incorporating latency into the model.

 Next we create the state using these calculations and feed it to the solver that returns the control inputs for the vehicle wrt the state and the coeffs we pass to the solver.

 ## MODEL PREDICTIVE CONTROL:

 MPC reframes the problem of following a trajectory as an optimization problem. The solution to the optimization problem is the optimal trajectory that the car follows. MPC involves simulating different actuator inputs, predicting the resulting trajectory and selecting that trajectory with the minimum cost. I chose the cost coeffs such that the the car is able to go around the track as smooth as possible. The cost function is implemented in the FG_eval class. I started with keeping rest of the coeffs in the cost calculation as 1 and focused only on the cte and epsi coeffs to let the car at least drive the entire track first, though not smoothly. Then I played around with the effects of the coeffs of the actuators and the coeffs of thier rates of change to get the car running as smooth as possible.

 In this method, the cost function is minimized at each step. At every time step the entire optimization problem is solved and cost is calculated.

 ## RESULTS

 With the final tuning, the car was able to drive upto a speed limit of 60mph. Although further tuning can make it go at higher speeds. I tried at 70 mph with current parameters and at sharp turns sometimes the car would go out of track. Future work can include doing this.

---

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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

