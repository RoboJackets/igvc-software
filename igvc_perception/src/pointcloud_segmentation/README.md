# Real-Time Clustering for Lidar Point Cloud Data

This repository contains a point cloud clustering package that clusters raw point cloud data into sevral objects.
The clustered objects can be used to identify depth distance between the robot and the objects near the robot for obstacle avoidance.
Region growing [1] and Euclidean clustering segmentation [2] are two types of clustering algorithms that clusters raw point cloud data in this package. 
This system uses the [PCL Library](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html) and a part of the PCL tutorial listed in the reference section.

<h1 align="center">
<img src="https://github.com/RoboJackets/igvc-software/blob/feat/pointcloud_clustering/igvc_perception/src/pointcloud_segmentation/example/example.png" width="45%" /> <img src="https://github.com/RoboJackets/igvc-software/blob/feat/pointcloud_clustering/igvc_perception/src/pointcloud_segmentation/example/raw_lidar%20points.png" width="45%" />
</h1>

## Folder Structure 
+ **clustering.cpp**: implements a publisher and a subscriber for running clustering algorithms.
+ **clustering.h**: defines a header file for clustering.cpp
+ **ground_filter.cpp**: implements a ground plane segmentation algorithm to remove the ground plane
+ **ground_filter.h**: defines a header file for ground_filter.cpp
+ **utils.h**: defines utility functions used in the clustering.cpp

## Build Instructions 

1. Download a bag file (e.g. JESSII_RUN_SUN_2_2019-06-10-15-30-13.bag) from the robojacket cloud
2. Modify `ptseg.launch` and add your file path
3. Run the following command to visualize the point cloud segmentation `roslaunch igvc_perception ptseg.launch` 

## Reference 
[1] Region Growing Segmentation (https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html) 

[2] Euclidean Clustering Segmentation (https://pointclouds.org/documentation/tutorials/cluster_extraction.html#)
