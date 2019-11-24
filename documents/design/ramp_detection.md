# Ramp Detection

 *Issue #563*

 **Author:**
 Monica Gupta

 ## The Problem

The problem is ramp detection, so that the robot does not classify the ramp as an obstacle and moves over it.
 
The end goal is to make sure the ramp isn't detected as an obstacle and to make sure that the white lines on the ramp are detected correctly so that the robot doesn't move off the side of the ramp.

 ## Proposed Solution

Try different approaches to detect the ramp using the non-ground LIDAR output. One idea is to use surface normals and detect an L shape.

Steps to be taken:

 -Write a node to detect the ramp and visualise how the detection works in Gazebo.

 ## Questions & Research

Research involved:

-Learn how to work with LIDAR output. Learn general object detection concepts related to LIDAR perception.

 ## Overall Scope

 ### Affected Packages

 Changes to code in the folder: IGVC Perception

 ### Schedule

 Subtask 1 (present - Dec 14): Learn how to use rviz, gazebo, use the output of LIDAR.

 Subtask 2 (Dec 14 - Jan 3): Make sure ramp detection works for at least a few cases.

 Subtask 3 (Jan 3 - Jan 15): Make ramp detection robust.

 Code Review (Jan 15): Hopefully all the code works and will be ready for review!