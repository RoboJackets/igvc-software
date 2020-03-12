# Fine Tuning TEB Parameters

*Issue #628*

**Author:**
- Amy Yao

## The Problem

The problem I am trying to solve is to fine tune the timed-elastic-bands control parameters for the
robot.

## Proposed Solution

- I will first watch more videos and read more articles abut timed-elastic-bands control to make sure
  that I understand how TEB works. I will learn what the appropriate values for each paramater
  are and what will changing each of the variables do.
- Then, I will try to use rviz and rqt_reconfigure to adjust, test, and tune the TEB parameters in the .ymal file.
- If all the parameters seem to be tuned, I will then make changes to the values in the file, and test
  on the robot if possible.

## Questions & Research

I need to research more about model predictive control and timed-elastic-bands in order to solve this issue,
and I also need to read more about teb_local_planner on ROS Wiki.

A preliminary list of resources that I will be using include:
https://www.mathworks.com/videos/series/understanding-model-predictive-control.html
https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7331052&tag=1
http://wiki.ros.org/teb_local_planner/Tutorials/Setup%20and%20test%20Optimization
https://www.youtube.com/watch?v=e1Bw6JOgHME

## Overall Scope

### Affected Packages

- The values in teb_local_planner_params.yaml will be tuned and changed accordingly.

### Schedule

Subtask 1 (1-2 meetings): Research and learn more about TEB and teb_local_planner.

Subtask 2 (1-2 meetings): Use rziv to try to tune the parameters.

Code Review/Subtask 3 (1 meeting): Try to test the tuned parameters on the robot, and adjust the values if needed.
