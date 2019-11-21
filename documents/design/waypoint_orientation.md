# Final orientation of waypoint for TEB is wrong

*Issue 596*


**Author:**
- Matthew Hannay

## The Problem

Currently the waypoint programming gives each waypoint a goal orientation of 0 degrees. This
means that at each waypoint the robot turns to face 0 degrees before moving on.

## Proposed Solution

The solution to this problem would be to change the message type of `/waypoint` from `PointStamped`
to `PoseStamped`, and then make the action server (or itself) send the pose directly instead of a point with 
an orientation of yaw = 0.

To calculate the intended waypoint orientation, the program will calculate the orientation that
points the robot towards the next waypoint. For the final waypoint, the final orientation should
correspond to one with yaw = 0.

Another aspect that can be fixed is to merge the `action_server.cpp` and `waypoint_source.cpp` into a
new node called `navigation_client.cpp`. Since the two files are solely dependent on each other, 
their functionality can be merged into a single file. 

There will be a parameter called `reading_from_file` that defines if the node will only send waypoints 
from a file or from rviz. If true, it will only send waypoints from a file and ignore any waypoints sent
by rviz. If false, it will not read from any file and only send waypoints from rviz.

The `navigation_client.cpp` node would:
- check if it reading waypoints from a file with the parameter `reading_from_file`

If reading from a file
- read waypoints from the path defined by the parameter `path` 
- create a list of `PoseStamped` which correspond to each waypoint
- use `sendGoalAndWait()` to send the waypoint to the navigation server
- when navigation is finished, send the next waypoint, repeat

If reading from rviz
- subscribe to rviz waypoint topic
- use `sendGoal()` in callback function

## Questions & Research

None.

## Overall Scope

### Affected Packages

- `waypoint_source.cpp`
- `action_server.cpp`

### Schedule

Create code to find correct orientation for waypoints : 11/27

Modify files that reference waypoints to use correct message type: 11/27

Code Review : 11/27
