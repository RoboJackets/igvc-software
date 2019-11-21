# Add Noise to Simulation

*Issue #552*

**Author:**
- William Wilson

## The Problem

The problem I am trying to solve is the lack of noise in the gazebo simulation. In order to
have a more realistic simulation and ground truth pose, noise is needed. In the real world,
sensors output noisy data, which we must work through to get an accurate position of the
robot. Currently, the simulation does not account for that. In the end, there will be noise
added to the ground truth pose, and this noise can be set in the gazebo launch files.

## Proposed Solution

I am going to simulate noise for the x and y position of the robot via random Gaussian noises
(random normal distribution) in the ground_truth node.

- Add Gaussian distribution randomizer
- Use randomizer to add noise to pose.position.x and pose.position.y
- Add randomizer's standard deviation value to a launch file
    - changing this value will allow the user to change the amount of noise generated
    by the simulated sensors)

## Questions & Research

I need to learn how Gaussian distribution works. Here's a C++ article on that:
www.cplusplus.com/reference/random/normal_distribution/

I also need to re-learn how launch files work in terms of which file I will add the parameter to.

## Overall Scope

### Affected Packages

- igvc_gazebo
    - ground_truth/main.cpp
    - launch

### Schedule

Subtask 1 (11/24/19): Create a randomizer and use it to add noise to x and y

Subtask 2 (12/01/19): Add a parameter so the amount of noise can be set in the launch files

Code Review (12/02/19)
