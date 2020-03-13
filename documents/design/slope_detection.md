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

For more information on the `elevation_mapping` library:
* github page: https://github.com/ANYbotics/elevation_mapping
* relevant paper: https://www.research-collection.ethz.ch/handle/20.500.11850/272110

The basics is that each cell is updated using a Kalman filter, taking into account the uncertainty of the data.
The pointcloud is transformed to the odom frame, and then fit into cells based on their `x` and `y` values, with
the height being their `z` value. There is an additional functionally to smooth over the elevation map, but we found
it was too inefficient for what we wanted to use it for.

A possible alternative to using the `elevation_mapping` library would be to make our own library. This would allow us
to remove our dependencies of `kindr_ros`, `kindr`, and `elevation_mapping` itself. However, the library handles a lot
of complex math, so its hard to recreate it without extensive knowledge on the subject. So we stuck to using the library.
