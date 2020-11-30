# Motion and Path Planning for Self-Driving Vehicles

One of the main purposes of a self-driving vehicle is to be able to arrive to a goal location without human 
intervention. To achieved it the vehicle must be able to figure out a route to arrive to the goal
position(Path Planning) and should be able to navigate the obstacles(Motion Planning) to arrive safely to the expected
position. For example, if the vehicle wants to arrive to your house, it should first use the GPS and find the shortest
route. After this has been calculated it is required to get context information such as lanes and other vehicles
to navigate safely to your house. https://arxiv.org/pdf/1604.07446.pdf

On this project we have implemented a planner that a allows the simulated vehicle to navigate through a three lanes
highway, avoiding: Collide with other cars, exceed speed, acceleration and/or jerk, and maintaining the lane until
is safe to maneuver.

## Change Lanes

The [planner::change_lanes](src/planner.cpp)  method uses a state machine to decide what maneuver must be executed. 
Using the Frenet coordinate system the vehicle calculates a safe path to drive through traffic.
1.	Uses sensor fusion to identify cars that are on the same lane, if a vehicle is detected on the same lane (line 121)
2.	When a car is detected first checks that is possible to use the left lane to overpass(line 132). It checks that 
there are not cars in front or behind within a safe distance(including acceleration)
3.	After checks that is possible to use the right lane to overpass(line 132). It checks that there are not cars in 
front or behind within a safe distance(including acceleration)
4.	Finally, if a car is detected to be to close the car slows down, it is possible to turn right it does, 
if not try to use the right lane (line 155)

## Waypoint Planner

The [planner::waypoint_planner](src/planner.cpp) method verify if is safe to drive using the change_lanes method and
drives using the map coordinates.
1.	Gets the car position and map waypoints in S,X,Y (line 31)
2.	Calculates the yaw (line 47)
3.	Convert the frenet points into map coordinates (line 55)
4.	Calculating with the actual lane, the next points to navigate 
5.	Transform to vehicle local coordinates (line 69)
6.	Smooths the navigation, yaw and path using the spline library (line 78)
7.	Finally converts back to the x, and y coordinates to be pass to the vehicle control system (line 90)




