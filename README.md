# Path Planner Highway Driving
Path planner that creates smooth, safe trajectories for the car to follow on a highway.

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
## Introduction
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH 
of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse
map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, 
which means passing slower traffic when possible, note that other cars will try to change lanes too. 
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. 
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should 
not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation Details
The project contains a [main.cpp](main.cpp) file that has the task to read the inputs and provide the outputs to the 
simulator. On the [planner.cpp](planner.cpp) can be found the implementation for the highway planning solution.
For more details you can refer to the post 
[Motion and Path  Planning for autonomous vehicles](https://advt3.com/posts/path_motion_planning.md)
 
## Basic Build Instructions
- **Clone this repo**
- **Make a build directory:** mkdir build && cd build
- **Compile:** cmake .. && make
- **Run it:** ./path_planning.

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
* [Eigen 3.3](eigen.tuxfamily.org/index.php) 
    git clone https://gitlab.com/libeigen/eigen.git
* [Cubic Spline interpolation in C++](https://kluge.in-chemnitz.de/opensource/spline/)

## Setup
### Ubuntu Linux
To install the required libraries run [install-ubuntu.sh](install-ubuntu.sh) script.

### Docker
In order to install the complete development environment the provided dockerfile can be used.

To build, compile and run the code we use a [docker image](Dockerfile) together with CLion.

- To build the image run

        docker build -t dev/env .
        
- To run the image

        docker run -p 127.0.0.1:2222:22 -p 127.0.0.1:4567:4567 --name particle-filter-env --rm dev/env 

The code can be copy using ssh, then use cmake to setup and make to build. Finally run the particle_filter executable.

For more details of the Clion integration go to the post:
 
[Using Docker with CLion](https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/)

## Contributing
[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)

## References
- [Project Definition](https://github.com/udacity/CarND-Path-Planning-Project)
- [Requirements](https://review.udacity.com/#!/rubrics/1971/view)
- [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) 

## License
MIT License Copyright (c) 2016-2018 Udacity, Inc.