# Mapping Refactor Design Document

**Authors**:
- Oswin So

## Motivation
With the decision to refactor our navigation stack from our current custom framework to
[move base flex](http://wiki.ros.org/move_base_flex)
(see zenhub epic [here](https://github.com/RoboJackets/igvc-software/issues/479)),
we need to refactor our navigation system to work with move base flex.

Because of our decision to use [teb local planner](http://wiki.ros.org/teb_local_planner)
which relies on [costmap2d](http://wiki.ros.org/costmap_2d) for mapping, we need to port
our current mapping logic to `costmap2d`.

## Background
Currently, our mapping system is handled by the `mapper` node in
[igvc_navigation/src/mapper](../../igvc_navigation/src/mapper).

The mapper node has multiple responsibilities including:
- Applying a series of filters to the incoming pointcloud in order to obtain points
representative of an obstacle to be put into the map
- Performing RANSAC on the pointcloud to identify the ground plane
- Taking in a segmented image from the neural network and projecting pixels representing
lines onto the identified ground plane
- Identifying empty space by finding "holes" in the occupied pointcloud, as well as
performing raycasting on occupied points to get the empty space between the sensor and the
occupied point
- Performing occupancy grid mapping (recursive bayesian filter) separately for
LiDAR obstacles (barrels) and lines

The `mapper` node is implemented using the octree provided by the
[octomap](http://wiki.ros.org/octomap) package as the backing data structure.

## Requirement
1. The mapper should be able to map both barrels (LiDAR) and lines (camera)
2. The mapper should perform occupancy grid mapping (recursive Bayesian filter)
3. The mapper should be part of the `costmap2d` framework in order to interface
with `teb_local_planner` through the use `costmap2d` layers

## Design

### Pointcloud filtering
To remove the responsibilities of filtering the pointcloud from the mapper, a
[nodelet](http://wiki.ros.org/nodelet)
`pointcloud_filter` will be created that (for now) performs all the filtering that
the current mapper does such filtering for distance and extracting empty space, which then
publishes that information to make the mapper's functionality more single responsibility.
For now, a flat ground plane will be assumed, since there is [an issue for a better
barrel segmentation algorithm](https://github.com/RoboJackets/igvc-software/issues/474).

### `OccupancyGridLayer`
A `OccupancyGridLayer` class that extends the `costmap_2d::CostmapLayer` will be created
in order to introduce occupancy grid mapping functionalities as a base class that the barrel
and line layers can extend off of to reduce duplicate code.

This base class should implement methods to:
- Insert a pointcloud as either occupied or free
- Allow for the implementation of a sensor model that changes the probabilities of the measurements
as a function of the position of the point

### `BarrelObstacleLayer`
A `BarrelObstacleLayer` class that extends the `OccupancyGridLayer` will be created.
This layer will have the responsibilities of:
- Subscribing to the output of the pointcloud filter
- Calling the `OccupancyGridLayer` methods that perform the Bayesian filtering on the
underlying map to insert the occupied and free points
- In the future, this layer may change its interal representation to store coordinates
of individual barrels (x, y, θ) as it will lead to a cleaner map.

### `LineLayer`
A `LineLayer` class that extends the `OccupancyGridLayer` will be created.
This layer will have the responsibilities of:
- Subscribing to the output of the segmented image projection node.
- Calling the `OccupancyGridLayer` methods to insert lines and non lines

## Overall Scope
- Expected manpower: 1 / 2
- Expected time: 3 weeks
