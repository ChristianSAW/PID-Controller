# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Approach and Results
In this project I implement a PID controller to drive a car around a race track.
The simplest way to complete this objective was to us a single PID controller for
the steering value input to the car. The error for this PID was Cross Track Error
(CTE). To satisfy the criteria of the vehicle not leaving the track, Approach 1
works well. I considered 2 additional approaches to attempt to increase the possible
driving speed while still maintaining stability.

### Simple & Safe PID (Approach #1)
At low speeds (~25-30 mph), a simple PID controller on the steering value of the
sufficiently keeps the vehicle in check. The final setup was as such:
Steering Value = -K_p*CTE - K_d(CTE - CTE_Prev) - K_i(cumulative_sum(CTE)).

#### Effect of P:
The proportional term, K_p drives the error term, CTE to zero. It accounts only for
the current value of the error. The larger the value,the faster the error is driven down.
The consequence of this is that large P values will make the error oscillate about a set point.
To account for this, we add a derivative term.

#### Effect of D
The derivative term, K_d helps overcome the overshoot caused by the proportional
control by accounting for the change in direction and rate of error. The derivative
control considers the current error and the previous error, or rather the change
between the two.

#### Effect of I
The integral terms, K_i helps account for the systematic bias due to calibration
or hardware or modeling) by adjusting the value which the error term oscillates
about. The integral control accounts for the cumulative sum of the error.

#### Tuning Hyper Parameters
I initially guessed the hyper parameters based off the ones used in the lesson.
Initial Guess: [K_p,K_d,K_i] = [0.12,4.3,0.002]. Rather than using twiddle, I was
able to manually tune these through trial and error until I got a satisfactory result.
* While tuning, I found that using a large P value e.g. 0.5 resulted in significant
overshoot that could not be overcome by the derivative control. Essentially, a large
P value made the controller incredibly sensitive to error and the car would never
recover from beginning oscillate and end outside the track. A P value that was too small
e.g. 0.01 would result in a controller that could not correct itself in time and
the car would go off the track at the first turn.
* 

### PID of Steering Value & Throttle (Approach #2)

### Linearized Steering Value PID (Approach #3)


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
