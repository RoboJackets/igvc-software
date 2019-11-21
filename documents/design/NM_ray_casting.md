# Stop raycasting at first obstacle

*Issue #543*

**Author:**
- Vikram Tholakapalli

## The Problem

The current raycasting continues until the max range. It should stop at the first obstacle in the map if it is within the raycasting radius (as that would also result in a "miss" because it is within the minimum range).
Essentially it should extend our Lidar range into the minimum range, and if there is an object in the minimum range then make sure the scan shows that the arc behind that object is blocked. This is important because right now if an object is closer than the LiDAR range, the LiDAR thinks that there is no object and there is open space in that region which influences the path planning.



## Proposed Solution

1. Before we start scanning from the endpoint to startpoint we need to make sure that there is no object within the minimum range.
1. So i'll need to add this code to the LiDAR layer, specifically insertFreeSpace.
1. Get the robot's current position
1. Iterate through the Line from the robot's origin to the minRange. If an occupied node exists in this range mark minRange as occupied.
1. I nothing in this range is occupied, then search from minRange to Endpoint.



## Questions & Research
- I also need a bag file I can test my changes on to see if my solution works.
- How do I get



## Overall Scope

### Affected Packages

- What parts of the software will you have to change (if any)?
    - I'll have to change the LidarLayer::insertFreeSpace method within lidar_layer.cpp
- Which packages are relevant to the success of the project?
    - the igvc_navigation layer



### Schedule

1. Subtask 1: Figure out to get bag files to work and fully understand the code base. (By Nov 27th)
2. Subtask 2: Get it to iterate from the robot's origin to the mapping point and figure out if this range is blocked. (By Nov 27th)
3. Subtask 3: If the range is not blocked scan from startpoint to endpoint. (By Dec 4th)
4. Subtask 4: Fix bugs + extra time for unexpected problems (By Dec 4th)

Code Review (Date): Dec 11th

*(School work will change over the semester and software often takes longer than you expect to write,
so don't worry if you fall behind. Just be sure to update this document with your progress.)*
