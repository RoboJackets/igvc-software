# Project Title

*Issue #Number*
571

**Author:**
- Jiajun Mao

## The Problem

In last year's IGVC competition, Jessie turned 180 degrees before starting traveling on the course. This is presumably caused by the current navigation model ignoring everything behind Jessie and default that nothing exists there. This caused the problem with the path planning algorithm thinking that the shortest path is to walk backward to the place where "no line exists".

## Proposed Solution

- The problem could potentially be solved by creating a semi-circle behind the robot with the diameter of the distance between the left and right lines of the course and make the robot think that that is a part of the line defining the course. Thus the robot would not travel backwards.
- How to measure the location of the cone
    - Obtain the layered_costmap_ from the costmap2d::layer which contains a getPlugin method that will return a vector of **boost::shared_ptr``<Layer``>** with each shared_ptr pointing to a layer
    - From the vector of layers we can obtain the LineLayer representing the lines the robot is seeing around itself
    - detect in a small radius of that line whether a point belongs to the same line still exist, until reaching the endpoint of the line
    - when both endpoints of the line on the right and left are reached, use a grid-based search algorithm to draw a line between the two points (or a cone)
- Create a new layer in the 2D cost map and artificially insert points on the line/cone created above to act as an obstacle behind the robot
    - Create a new layer in igvc_navigation/mapper
    - Incorporate the new layer into the cost map
- Create a ROS service that generate the new cost2d map
    - call when service when the robot reaches a waypoint and at the start of the course
    - the service will also contain a remove method that will remove the cone created at the course start after the robot has reached the first way point [or some other specific period of time]


## Questions & Research

- ROS Service
    - Similar to subscriber and pubisher but is a one time call
    - written with a .srv file and a structure with response/request classes will be generated
    - ServiceClient calls a service with
        - ros::ServiceClient srvClient = nh.serviceClient<serviceType>("service_name");
        - srvClient.call(serviceInstantiation); [returns true if service calls with success and serviceInstantion is filled, and false otherwise]
    - Service contains a callback method used to process the request upon received. It is called with
        - ros::ServiceServer srvServer = nh.advertiseService(serviceType, <callback>);

- Costmap2d layer
    - A map that indicates discretized information about the world the robot is in
    - consist of a flattened array that represent a two-dimensional map
    - contains useful functions such as:
        - void GridLayer::updateCosts - update the cost in the master grid map according to the value in this layer
        - void GridLayer::updateBounds - expand the cost map layer to include new cells that the robot is in

## Overall Scope

### Affected Packages

- igvc_navigation/src/mapper


### Schedule

Subtask 1 (11/24/2019): Finish generating the line/cone

Subtask 2 (11/28/2019): Modify mapper.cpp to insert the new layer

Code Review (12/2/2019): Most likely I would have everything figured out by then and ready for review. Hopefully it could be sooner.
