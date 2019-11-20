# Final orientation of waypoint for TEB is wrong

*Issue 596*


**Author:**
- Matthew Hannay

## The Problem

Currently the waypoint programming gives each waypoint a goal orientation of 0 degrees. This
means that at each waypoint the robot turns to face 0 degrees before moving on.

## Proposed Solution

The solution to this problem would be to change the message type of /waypoint from PointStamped 
to PoseStamped, and then make the action server (or itself) send the pose directly instead of a point with 
an orientation of yaw = 0.

To calculate the intended waypoint orientation, the program will calculate the orientation that
points the robot towards the next waypoint. For the final waypoint, the final orientation should
correspond to one with yaw = 0.

The pose calculation will be something like:

geometry_msgs::PointStamped Waypoint1, Waypoint2;
double angle = std::atan2(Waypoint2.y - Waypoint1.y, Waypoint2.x - Waypoint1.x);
geometry_msgs::PoseStamped waypoint_pose;
geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(angle);
waypoint_pose.pose.point = Waypoint1;
waypoint_pose.pose.orientation = quaternion;
waypoint_pose.header = Waypoint1.header;

Another aspect that can be fixed is to merge the action_server.cpp into waypoint_source.cpp. Since
the two files are solely dependent on each other, their functionality can be merged into a single
file. This would mean that waypoint_source.cpp would have a MoveBaseClient object inside of it.

A parameter will also be added which sets if the waypoints come from a file or are defined by the user.

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
