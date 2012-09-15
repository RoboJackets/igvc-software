This is the where all of the pathplanning currently goes on. The actual 
implementation of this a real-time instantaneous modified version of
potential fields that moves the robot perpendicular to nearby obstacles.

An important function in potentialfields.cc to know is dropWaypoint. This 
is outside interface that allows users of the object to inject the current 
position of the robot into the path-planning. It should be invoked just
before the function call to getVectorMotor. getVectorMotor takes in a bunch
of things including the obstacle probability map and outputs the suggested
output to send to the motors. There is another interface called 
getCompleteVector, but this function (running A*) was far too slow to be 
useful.

For planning purposes, the class uses the waypoints in gps_points.hpp, in
order as the goals. Note: this is a horrible way of doing it, as we found
out at competition. It's very difficult to put the points in correctly
without screwing it up. I would recommend another solution such as a xml
file or even a gui for entering in gps waypoints.

CommonVecOps is a collection of functions used in potentialfields. If you
like, you can find the c-libraries that do some of the same things, 
although some of the functions are specifically mapping to domains 
specific to this. A better solution might be to have a seperate class for
doing these conversions (I believe it is already partially implemented in
roxi/RobotPosition/Coordinates.cpp).

Many other solutions to pathplanning are possible, but this is the one we
used at competition, and it seemed to work out pretty well. 

Let me know if you have any questions about this code (since I specifically
wrote this), or any other code (which I either helped write, edited, or 
have after-the-fact knowledge of).

Kenneth Marino
kmarino3@gatech.edu
