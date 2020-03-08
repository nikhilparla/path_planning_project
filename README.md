# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Progress
*Initial commit*
 - Installed the uWs library and header files in the local machine
 - Changed the eigen calls in main.cpp to reflect the headers in local usr/lib insted of the folders in src
 - Deleted the eigen folder from src
 - Compiles and runs successfully. Car doesnt move yet.

*Stays in the lane and drives*
 - Made the initial changes according to the project QandA  
 - Car moved in straight line with the generated coordinates

*ego car runs smoothly in its lane*
 - Included the spline.h header only package. This can generate polynomial for line fit  
 - Created waypoints at 30,60 and 90 ahead and fit next 50 points
 - Resued whatever points were left over from previous case
 - Car now runs smoothly in the same lane. Jerk still present since 0-49.5 in one instant

*gradual change of speed*
 - Taking into account sensor fusion data. Looping through all the car objects a checking which ones are in our lane  
 - For the ones in our lane, check the s value if it is in front and less than 30 mts ahead and reduce speed
 - If there is no car, increase the speed back to highest. Keep checking this every loop

*lane change logic added. few collisions*
 - if you are reducing speed, check the lane traffic on left and right lane of ego lane
 - check corner cases for lanes 0 and 2
 - If there are no cars for 30mts in front and 10 int he back, good to change lanes
 - colliding with cars right next to it in some cases. Assume it is because of the prev end value of s

*collisions reduced*
 - changed logic to reflect the previous end s value
 - collisions reduced drastically and car changes lanes smoothly
 - Max distance without collisions 1.43 miles

*no collisions. finished track*
 - One change in the reduce speed calculation to check from prev end s to curr s value made car go safer

#### Reflection
As discussed in the project QandA, Spline header-only library has been used to generate the path. Every cycle of 20ms, we check the lane and fusion information and generate waypoints for the next 30,60 and 90 meters. \
A brief explanation of the code -
 - With the sensor fusoin information, check if any vehicle present in our lane 30mts out
 - If so, reduce speed and check vehicles one lane to the left and one to the right if present
 - If not free, stay in the lane and reduce speed. If lanes free, prefer faster left lane
 - Now to generate path, first get the last two points of previously generated path to get the angle on which the car was previosly heading in
 - Get also the xy coods of the points at 30, 60 and 90 mts out front
 - With the 5 points obtained, fit a polynomial with spline header functions after shifting the frame to car's reference
 - With the polynomial for the path calculated, we need to get few waypoints for the ego car to follow
 - First get the points that were leftover from the prevoius state so that it helps in our transition
 - A total of 50 points need to be generated and for the rest of the points needed after adding the ones from the previous path, we have to space them evenly so that the car doesnt exeed the speed limit
 - To get the points on the calculated spline, we used a target distance of 30mts out front and the calculated the xy coods of those in car's frame
   - The car visits a points every 20ms
   - N pieces times 0.02s times velocity = distance to travel
   - With this we get the number of pieces needed to generate and get the trajectory
   - higher the velocity, smaller the piece
 - Once we geenrated the full 50 points for the car to follow, we transformed then back to map frame and sent to the simulator

Everything below this line are the original instructions for the project. 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
### Simulator.
Download the Term3 Simulator from [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

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

["sensor_fusion"] A 2d vector of cars and then that car's [
  0 - car's unique ID, 
  1 - car's x position in map coordinates, 
  2 - car's y position in map coordinates, 
  3 - car's x velocity in m/s, 
  4 - car's y velocity in m/s, 
  5- car's s position in frenet coordinates, 
  6- car's d position in frenet coordinates] 

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


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


