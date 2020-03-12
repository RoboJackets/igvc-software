# Slope Detection Design Document

**Author: Matthew Hannay**

## Motivation
The old `lidar_layer` worked for detecting barrels, but caused issues when approaching the ramp.
The ramp would be seen as an obstacle, which meant we could no go past it.

With slope detection and `traversability_layer`, a costmap will be generated based on the slope at any given point.
For barrels, the steep walls will be detected as untraversable.
For the ramp, the edges will be too steep to traverse, but the center will be detected as traversable.

## Requirements
Given an input pointcloud:
1. Filter out far away points and points behind the robot.
2. Add the points to an elevation map.
3. Find the gradient at every point in the elevation map.
4. Generate a costmap based on that gradient.

## Design

`pointcloud_filter` will filter out any points that are far away or behind the robot.

With the `elevation_mapping` library, we can make a grid map with elevation in each cell. A node is created that
subscribes to the filtered pointcloud and publishes an elevation map.

With the `grid_map` native filtering functionality, we can run the elevation map through several filters to give us
the slope we desire. The filters are:
* Remove unneeded grid map layers (everything but `elevation`)
* Perform a boxblur to fill in some small holes
* Compute surface normals
* Compute slope from surface normal

`traversability_layer` subscribes to the slope grid map, and uses log-odds to update the traversability probability
for each cell in the costmap. If the slope is above a certain threshold, the cell is more likely to be untraversable.
