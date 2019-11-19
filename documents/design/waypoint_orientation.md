# Final orientation of waypoint for TEB is wrong

*Issue 596*


**Author:**
- Matthew Hannay

## The Problem

Currently the waypoint programming gives each waypoint a goal orientation of 0 degrees. This
means that at each waypoint the robot turns to face 0 degrees before moving on.

## Proposed Solution

The solution to this problem would be to change the message type of /waypoint from PointStamped 
to PoseStamped, and then make the action server send the pose directly instead of a point with 
an orientation of yaw = 0.

To calculate the intended waypoint orientation, the program will calculate the orientation that
points the robot towards the next waypoint. For the final waypoint, the final orientation should
correspond to one with yaw = 0.

## Questions & Research

None.

## Overall Scope

### Affected Packages

- waypoint_source.cpp
- action_server.cpp

### Schedule

Create code to find correct orientation for waypoints : 11/27

Modify files that reference waypoints to use correct message type: 11/27

Code Review : 11/27
