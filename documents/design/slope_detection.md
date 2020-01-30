# Slope Detection and Traversability Analysis

*Issue #629*

**Author:**
- Matthew Hannay

## The Problem

Right now our robot cannot cross the ramp. This is because it is not able to detect if something above the
ground is an obstacle or something that can be driven on.

## Proposed Solution

The solution I've thought up involves a more thorough pointcloud filtering. Instead of just saying "anything
high off the ground is an obstacle", there will be a more complicated process to determine what is an obstacle.

* The robot will use the lidar to get a pointcloud of its surroundings.
* The point cloud will be filtered to remove points far away and behind the robot.
* The code will iterate over each point and find the height of the point. There will be a height-map that
stores the height at each square in the grid to a certain degree of confidence.
* For the occupancy-map, the code will calculate the slope at each point in the height-map to determine
if the terrain is able to be traversed.
    * The method of calculating the slope is to first find the normal vector of the surface at the point.
    Then we can find the gradient of the tangent plane using the normal vector. This gradient is the maximum
    slope at this point. If the gradient is above a certain threshold, the point will be set to "occupied".
* The occupancy-map will be sent to where it needs to go so the robot can navigate properly.

## Questions & Research
* How easy is it to determine the height of the points in the pointcloud? Is there variance we need to account
for? Do we need to do any transformations?

* What should the maximum gradient the robot can traverse along?

* Should we use the ANYBOTICS `elevation_mapping` library or create our own smaller one? `elevation_mapping`
is mostly geared towards legged robots and seems to be more complicated for that reason. I think I could
create a smaller and more optimized (and easier to read) piece of code that creates the height-map.

## Overall Scope

### Affected Packages

- Will modify the current pipeline for pointcloud filtering and occupancy-map adding.

### Schedule

Middle of Feb: Complete the code.

Sometime later: Test robot on slopes.
