# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

<!-- ## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html). -->

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


<!-- ## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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
still be compilable with cmake and make./ -->

<!-- ## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777). -->

## Model Documentation
### Finite state machine
The path planner is based on a finite state machine which determines the autonomous car's driving behaviour on the highway. The state machine is composed of the following six states:
* CS - state starts the car
* KL - state keeps car driving in the lane
* PLCR - state prepares lane change to the right
* PLCL - state prepares lane change to the left
* LCR - state changes lane to the right
* LCL - state changes lane to the left

<img src="FSM.jpeg"/>

### Cost function
Once the car started from CS to KL state, transition between states is conditioned by a multi-objective cost function. In general, the state machine will always move from the current state to the next state which has the lowest cost among all possible new states.
For a given state, the cost function evaluates the weighted sum of:
* Traffic ahead cost - further the car ahead in the intended lane, lower the cost
* Efficiency cost - computes the difference between real speed/speed limit plus the difference between intended speed/speed limit, larger the real speed and intended speed, lower the cost. The real and intended speed will be determined regarding traffic speed in the target lane, acceleration/deceleration limit and road speed limit.
* Maneuver cost - in terms of maneuver, lane change has a higher maneuver cost than keep lane while left lane change is preferred with a lower cost compared to right lane change
* Reach goal cost - when traffic is heavy around, the planner will search the least busy lane as the goal lane to reach, so closer to the goal lane lower the cost. This will help the car to get close to the fastest lane more quickly.

The weights associated with each cost are manually tuned to guarantee that ego car changes lane only when it makes sense.

In particular, transitions from KL to PLCL/PLCR, PLCL to LCL and PLCR to LCR are only possible when the safe lane change conditions are verified:
*  no collision with the car ahead in the current lane
*  no collision with the car ahead & behind in the new lane

### Trajectory generation
The planner will then generate the associated trajectory to a new state using cubic spline interpolation method. There are three types of trajectories: keep lane, prepare lane change and lane change. Given the target lane of the new state, the lateral position d in the Frenet coordinate is defined as d = target_lane*4+2. Then for the same d position, I create three breakpoints with progressive s position from ego car's current position. The interval of s position between each breakpoint is a function proportional to ego car's target velocity with tuneable parameters specific to each type of trajectories. Those final parameters are iteratively tuned assuring respect of acceleration/jerk limitation and smoothness of all types of trajectory.

I append the three new breakpoints to the last two points of previous path and then convert them into local frame of the end point of previous path, the five points are base grid points to be interpolated using cubic spline.
By setting a target longitudinal position for the new trajectory, I can first interpolate the corresponding target lateral position from the base grid points and obtain the target distance in local frame of the end point of previous path. Knowing the target velocity, the target distance, I can easily get the spline points' interval in longitudinal direction. Finally I increment the longitudinal spline points and interpolate their corresponding lateral positions and switch back to global frame in order to complete the remaining part of the previous path.

<img src="path.jpeg"/>

The planner will realize the best state and send its associated path to the simulator.

### Results
The actual implementation of path planner is able to drive the car more than 65miles without incident

<img src="result.png"/>
