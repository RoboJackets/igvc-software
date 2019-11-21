# Project Title

*Issue #Number*
571

**Author:**
- Jiajun Mao

## The Problem

In last year's IGVC competition, Jessie turned 180 degrees before starting traveling on the course. This is presumably caused by the current navigation model ignoring everything behind Jessie and default that nothing exists there. This caused the problem with the path planning algorithm thinking that the shortest path is to walk backward to the place where "no line exists".

## Proposed Solution

- The problem could potentially be solved by creating a cone behind the robot and make the robot think that that is a part of the line defining the course. Thus the robot would not travel backwards.
- Create a new topic publishing a map with artificially created points and then feed that into the navigation 
    - Create a map with artificially inserted points and publish the map to a topic (maybe something like /cam/fakeback)
    - Create a subscriber in the navigation stack for the topic and interegrate the mapping into path planning cost map

## Questions & Research

- I probably just need to go through the input of navigation stack to make sure the specific format I am supposed to be publishing and the format for the name of the topic

## Overall Scope

### Affected Packages

- igvc_perception
- igvc_navigation


### Schedule

Subtask 1 (11/24/2019): Finish creating the map and publish it

Subtask 2 (11/28/2019): Modify the navigation stack to incorporate new map data

Code Review (12/2/2019): Most likely I would have everything figured out by then and ready for review
