# CarND3-P1 Path Planning

## Description

**This my 1st project result of Udacity self-driving car nanodegree (CarND) term 3. It's required to implement path planning of self-driving car based on behavior planning, trajectory planning and prediction of all vehicles. A vehicle simulator is provided to validate performance of path planning.**

**The following demonstrates vehicle moving in simulator based on designed path planning :** 
   
![alt text][image1]

* Udacity self-driving car nanodegree (CarND) :

  https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
  
* Udacity self-driving car nanodegree term 3 simulator :

  https://github.com/udacity/self-driving-car-sim/releases/

[//]: # (Image References)
[image1]: ./images/path_planning.gif


## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


## Behavior planner

The behavior planner is implemented in `main.cpp` line 312~419. If ego vehicle gets too close to ahead vehicle, it will try to change lane if nearby lane is available to make a safe change, or else it will keep lane. 

In lane changing, it will change to the one with farer ahead free space if there's more than one lane available or else change to the only one available. In lane keeping, it will decrease speed to prevent collision.

In the case ego vehicle is not too close to ahead vehicle, it will accelerate to reach speed limit and try to make lane change to keep center lane if center lane ahead free space is farer than current lane above 30 mile. The 30 mile constraint prevents frequent lane changing if there's ahead vehicles in both lane moving side by side.

The center lane keeping strategy comes from the idea of more opportunity to change lane and overtake other vehicle compared to inner/outer lane. 


How I estimated availability of safe lane change is based on relative `s` value of ego and other vehicle. In a candidate lane for lane changing, both nearest ahead and behind vehicles relative to ego vehicle in Frenet coordinate are estimated using ego vehicle data and `sensor_fusion` data.

In candidate lane, if nearest ahead vehicle is above 30 mile and nearest behind vehicle is above 20 mile relative to ego vehicle location, that lane is estimated having enough space for a safe lane changing and lane changing available flags are set (`left_available`, `right_available`).

## Prediction

The prediction is implemented in `main.cpp` line 270, 284~287, 306~310. 

Prediction of ego vehicle assumes ego vehicle goes the previous path, i.e. `car_s` is set as `end_path_s` in `main.cpp` line 270.

Prediction of other vehicle should also consider the previous mentioned time goes by. Therefore, besides `s` value obtained from `sensor_fusion`, distance traveled in the elapsed time should also be included, i.e. add other vehicle velocity times elapsed time in `main.cpp` line 284~287 and 306~310. 


## Trajectory generation

After all behaviors are determined, the trajectory generation is based on spline using 5 anchor points. In general case, the first 2 points are based on last 2 end points of previous path. If there's not enough previous path points, use current ego pose to extrapolate a previous point.

The last 3 points are 30 miles between each other in `s` value starting from current ego `s` value `car_s` + 30 miles, i.e. `car_s+30`, `car_s+60`, and `car_s+90`. `d` value is determined by the `lane` choice from behavior strategy. As time goes by, `lane` value may be changed based on behavior planner and a corresponding lane changing trajectory will be generated in trajectory generation.

All above 5 anchor points will be used to generate a spline function. 50 way points of trajectory are generated by specify a series of equal difference distance `x` values according to how fast we want to drive and then refer spline function to find out `y` values. After all way points are generated, they are sent back to simulator to control ego vehicle.

The trajectory generation is implemented in `main.cpp` line 449~560. 


## Reflection

Further improvement can be adressed on path prediction of other vehicle based on a series of location, velocity and acceleration. Also, finite state machine can be introduced to make a sophisticated behavior.


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
