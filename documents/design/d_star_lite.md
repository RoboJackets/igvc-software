# Project Title

*Issue #567*
(What GitHub issue is associated with this project)

**Author:**
- Christopher Lo

## The Problem

Currently, Jessii, our IGVC robot, uses A* in order to plan its path through the obstacle
course. This is all done with ROS package move_base_flex, which serves to allow the robot to
utilize an enhanced version of the planner, controller, and recovery plugin ROS interfaces. 
However, there is a better path planner algorithm, D* lite, which builds upon the Lifelong
Planning A* (LPA) algorithm and D* algorithms. Thus, to resolve this issue, I will be replacing
the currently used A* algorithm with the better D* lite algorithm in order to improve Jessii's
path planning on the obstacle course.

## Proposed Solution

- Read a research paper on D* lite by researchers Sven Koenig and
Maxim Likhachev in order to understand the technical details behind Lifelong Planning A* and D* Lite for
robot navigation in unknown terrain, which essentially includes goal-directed navigation and the mapping of
unknown terrain, concepts that will be useful to understand when implementingthe D* lite algorithm. 

- Read up on move_base_flex, a ROS package responsible for the planning, control, and recovery features
for Jessii.

- Read up on the parameters used by the method that will contain the D* lite algorithm implementation within
the software base.

- Create an implementation using the parameters and concepts learned from the D* lite research paper.

- Run tests using a simulator and see if the planner works better than the A* algorithm currently in place.

## Questions & Research

Currently, I do not know much about D* Lite (which is why I will be reading the research paper to extract
information that will be essential for creating a working implementation of the algorithm). In addition,
I will need to read up on A* and LPA (Lifelong Planning A*) in order to understand why D* Lite is better than
these algorithms and how I can seamlessly reap the benefits.

Link to D* Lite paper: https://aaai.org/Papers/AAAI/2002/AAAI02-072.pdf

## Overall Scope

### Affected Packages

The parts of the software that I will have to change are the move_base_flex wrapper_global_planner.h file,
which contains the makePlan() method that will ultimately be responsible for holding my D* lite algorithm 
implementation. I will also be editing the planner core file within the navigation folder in order to render 
a map of the unknown terrain that Jessii will be crossing and using to calculate a path with the D* lite 
algorithm.

### Schedule

Subtask 1 (11/24/2019): Read up on the D* Lite research paper. (11/24/2019)

Subtask 2 (11/31/2019): Read up on move_base_flex package and methods used in the body of the method containing the 
D* Lite algorithm. 

Subtask 3 (01/01/2020): Create an implementation of the D* Lite algorithm in the method. (01/01/2020)

Subtask 4 (01/10/2020): Run tests on the newly implemented D* Lite algorithm using a simulator or actual Jessii.

Code Review (1/15/2020): Code/result will be ready to review

