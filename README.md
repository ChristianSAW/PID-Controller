# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: Figures/approach_1_Figure_1.png "Approach 1 CTE vs Time"
[image2]: Figures/approach_1_Figure_2.png "Approach 1 CTE vs Steering Value vs Angle vs Time"
[image3]: Figures/approach_1_Figure_3.png "Approach 1 CTE vs Throttle vs Speed vs Time"
[image4]: Figures/approach_2_Figure_1.png "Approach 2 CTE vs Time"
[image5]: Figures/approach_2_Figure_2.png "Approach 2 CTE vs Steering Value vs Angle vs Time"
[image6]: Figures/approach_2_Figure_3.png "Approach 2 CTE vs Throttle vs Speed vs Time"

## Approach and Results
In this project I implement a PID controller to drive a car around a race track.
The simplest way to complete this objective was to us a single PID controller for
the steering value input to the car. The error for this PID was Cross Track Error
(CTE). To satisfy the criteria of the vehicle not leaving the track, Approach 1
works well. I considered 2 additional approaches to attempt to increase the possible
driving speed while still maintaining stability.

### Simple & Safe PID (Approach #1)
At low speeds (~25-30 mph), a simple PID controller on the steering value of the
sufficiently keeps the vehicle in check. The final setup was as such:<br>
```
Steering_Value = -K_p*(CTE) - K_d*(CTE - CTE_Prev) - K_i*(cumulative_sum(CTE)).
```
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

#### Tuning Hyperparameters
I initially guessed the hyperparameters based off the ones used in the lesson.
Initial Guess: [K_p,K_d,K_i] = [0.12,4.3,0.002]. Rather than using twiddle, I was
able to manually tune these through trial and error until I got a satisfactory result.
* **Tuning K_p:** I found that using a large K_p value e.g. 0.5 resulted in significant
overshoot that could not be overcome by the derivative control. Essentially, a large
P_p value made the controller incredibly sensitive to error and the car would never
recover from beginning oscillate and end outside the track. A K_p value that was too small
e.g. 0.01 would result in a controller that could not correct itself in time and
the car would go off the track at the first turn.
* **Tuning K_d:** Tuning K_d went hand-in hand with K_p, K_d had to balance the overshoot
without being to large. A large K_d value such as 10 would effectively drive the car off
the track as the controller counter steers the car too much. And, as mentioned before,
a K_d value too small such as 1.0 would not be able to combat overshoot.
* **Tuning K_i:** A large K_i value such as 0.1 would result in the vehicle simply
driving off the track because the bias is being over accounted for. A too small value
such as 0.00005 would no correct the offset and lead the car to be off center. In situations
of high speed and oscillations, this can be enough to cause the car to exit the road.

After manual tuning, I settled on these values:
[K_p,K_d,K_i] = [0.225,4.05,0.0005]

A graph of the CTE, Throttle, and Speed vs Time can be seen below.

![alt text][image2]

The following video shows the PID in action.

[![IMAGE ALT TEXT HERE](http://i3.ytimg.com/vi/VPekjcBFmn4/hqdefault.jpg)](https://youtu.be/VPekjcBFmn4)

### PID of Steering Value & Throttle (Approach #2)
To increase the speed of the car, I needed to increase my throttle correctly. The issue
here, however is that if the car is too fast, the steering control will fail, so the throttle
control must take into account the steering.

In this approach, I kept the PID on steering relatively the same and I combined 3 different PID controllers
to control throttle.
```
PID_1 = K_p,1*(E_speed) - K_d,1*(E_speed - E_speed_Prev)
PID_2 = -K_p,2*(CTE) - K_d,2*(CTE - CTE_Prev) - K_i,2*(cumulative_sum(CTE))
PID_3 = -K_p,3*(SV) - K_d,2*(SV - SV_Prev)
Throttle = PID_1 + PID_2 + PID_3
```
*Note, SV = Steering_Value, E_speed = Desired Speed - Actual Speed*

* The First PID controller was to simply to get the car to a certain speed. The proportional term
does this while the derivative term acts as a damper on the rate of throttle change. After manual tuning,
the values used were, [K_p,1; K_d,1] = [0.25; 2.0].

* The Second PID controller was to include the impact of CTE on throttle. Effectively, the terms here were used to
make the throttle take into account CTE in the same manner as the steering did. After manual tuning, the values used
were, [K_p,2; K_d,2; K_i,2] = [2.0; 20.0; 0.0005]. This is important as different speeds will affect CTE since the distance
traveled between each correction to speed and throttle will vary depending on the speed.

* Third PID controller was to reduce the throttle during high steering values so the car had time to correct itself.
After manual tuning, the final values were [K_p,3; K_d,3] = [4.0; 40.0].

**The Difficult Part**
The crux of the problem here is determining the appropriate relationship between
steering value and throttle when designing the controllers because these two values
affect each other when attempting to minimize CTE and E_speed. To minimize the CTE
and distance between your actual speed and desired speed, one must determine the
best way to make speed dependent on throttle and/or vice versa.

In this approach, steering was left as the independent variable with throttle being
the dependent. However, with the number of hyperparameters, it was difficult to tune the
controllers. While I did not use an optimizer here, to tune these properly, I would recommend
using one such as twiddle. To see twiddle implemented well, see either,
1. Jeremy Shannon's use of twiddle. Here, he updates the parameters after each lap completion
of the simulation. Be aware this results in him running the simulation for quite a while. He notes
that he let the car run for 500 laps to optimize the parameters he uses.
  * Link: https://github.com/jeremy-shannon/CarND-PID-Control-Project
2. Vivek Yadav's use of twiddle. Vivek updates parameters after an arbitrary number of
steps.
  * Link: https://github.com/vxy10/P4T2SCND_PIDControl

Note that the steering value parameters were slightly changed for this approach,
[K_p,K_d,K_i] = [0.15,4.05,0.0005].

The maximum speed achieved here while the vehicle was stable was 61.5 mph. During this run,
the vehicle stays on the track but it wobbles quite a bit and will cross the lane lines
at certain points. The average speed here is lower between 40-45 mph.

A graph of the CTE, Throttle, and Speed vs Time can be seen below.

![alt text][image5]

A graph of the CTE, Steering Value, Steering Angle vs Time can be seen below.

![alt text][image6]

The following video shows the PID of Approach 2 in action.

[![IMAGE ALT TEXT HERE](http://i3.ytimg.com/vi/0ahAKrdoMZI/maxresdefault.jpg)](https://youtu.be/0ahAKrdoMZI)

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
