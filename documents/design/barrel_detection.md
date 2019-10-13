# Barrel Detection Design Document

**Author: Joshua Viszlai**

## Motivation
Our current ground plane segmentation implementation uses a ransac-based plane segmentation algorithm from [pcl](http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php).

This approach assumes the ground is flat, which is not a viable assumption given the rough terrain we need to traverse. It also removes some control of the algorithm itself since we did not implement it. Therefore we need to come up with a new approach that removes the assumption that the entire ground is one flat plane, and one that we can ideally implement ourselves (for both added control and educational purposes).

## Requirements
Given an input pointcloud:
1. Distinguish ground points vs. non-ground points (barrels, trees, etc.)
2. Be robust to hills and rough terrain

## Design

A node will be created that subscribes to the `velodyne_pointcloud` topic and publishes to `ground` and `nonground` topics. This node will implement the algorithm presented in the barrel detection [research document](../research/barrel_detection.md).
 

