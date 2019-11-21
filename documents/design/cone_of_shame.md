# Project Title

*Issue #Number*
571

**Author:**
- Jiajun Mao

## The Problem

In last year's IGVC competition, Jessie turned 180 degrees before starting traveling on the course. This is presumably caused by the current navigation model ignoring everything behind Jessie and default that nothing exists there. This caused the problem with the path planning algorithm thinking that the shortest path is to walk backward to the place where "no line exists".

## Proposed Solution

- The problem could potentially be solved by creating a cone behind the robot and make the robot think that that is a part of the line defining the course. Thus the robot would not travel backwards.
- Create a new layer in the 2D cost map and artificially insert points to act as a cone-shaped obstacle behind the robot
    - Create a new layer and artificially create point behind the robot
    - Incorporate the new layer into the cost map

## Questions & Research

- I probably just need to go through the input of navigation stack to make sure the specific format I am supposed to be publishing and the format for the name of the topic

## Overall Scope

### Affected Packages

- igvc_navigation
    - src/mapper/fakecone.cpp [new]
    - src/mapper/mapper.cpp [modify]


### Schedule

Subtask 1 (11/24/2019): Finish creating the layer

Subtask 2 (11/28/2019): Modify mapper.cpp to insert the new layer

Code Review (12/2/2019): Most likely I would have everything figured out by then and ready for review. Hopefully it could be sooner.
