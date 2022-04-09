# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

The goals / steps of this project are the following:

Create a PID control algorithm to guide a vehicle around a track.
Tune PID hyperparameters so the vehicle smoothly follows the road, minimizing the cross-track error.
Implement a twiddle algorithm to continuously tune the hyper parameters.

## Project Introduction
This project implements a PID controller with twiddle optimization of the PID hyperparameters. 
The PID control algorithm runs based on the cross-track error (CTE), provided by the simulator, 
which is calculated using the vehicle's distance from the center of the lane. 
The CTE is then used in a series of formulas to calculate the proportional (P), 
integral (I) and derivative (D) values which together can be used to calculate the total error, alpha. Ultimately, 
the total error is used as a steering angle and throttle measurement that corrects the vehicle's position and speed 
until it reaches the center of the lane and maximum acceleration. The equation for total error (alpha) is shown below.

![img](./img/Screenshot%20from%202022-04-09%2013-59-45.png)

The first term (-taup * CTE) is the proportional component that allows the system to minimize the CTE, 
but it always overshoots the target resulting in an oscillation around the target. The second term 
(-taud * derivative of CTE) is the derivative term that counteracts the oscillation from P. 
Finally the third term (-taui * sum_CTE) is the integral term that corrects for the system if the PID controller 
is running parallel, but not equal to, the target.

## Hyperparameter Tuning
This section discusses how the hyperparameters, also known as tau, were chosen for each term P, I and D. When manual 
tuning a hyperparameter, sequentially tuning P, I and D keeping the previous setting.

**P** - This was tuned by staring at a small value and gradually increasing until the vehicle began swerving back and 
forth, showing the oscillation of the P term. I chose 0.2 for the steering PID.

**I** - This value was largely unused because after tuning P and D, the vehicle stayed in the center of the lane. 
I chose a value of 0.001 in the steering PID to minimize I's influence while still allowing for improvement through 
twiddle as the vehicle is driving.

**D** - This was tuned with the P value set, then I increased the D value until the oscillation stoped. 
I chose 2.0 for the steering PID to ensure a smooth ride.

## Twiddle
The twiddle algorithm continuously tunes the PID controller's hyperparameters by analyzing the cross-track error and 
keeping track of the smallest CTE. If the vehicle starts to veer and the CTE increases, the twiddle algorithm will 
incrementally increase or decrease the hyperparameters until the CTE is minimized.

## Speed control
Added main.cpp a speed limiter depending on the steering wheel angle in `main.cpp`. I chose 0.4 for the gas pedal 
and then subtracted the steering angle value times 0.3.

## Final PID settings
The car drove in the simulation for about 3 hours, and the following parameters were found using the Twiddle algorithm:  
**P** = 0.317452  
**I** = 0.00105314  
**D** = 2.82601  
These values have been added to `main.cpp`.

## Results

![gif](./img/example.gif)

Full video: https://youtu.be/tqUNd39fuNs

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

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

