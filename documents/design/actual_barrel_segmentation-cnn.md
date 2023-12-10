# Actual Barrel Detection (Neural Network) Design Document

**Author: Zhiyan Zhou (Frank)**

## Motivation
I implemented a version of the Bareel Detection algorithm using Clustering and Ransac. While this algorithm is relatively simple, there are many problems with it. For instance, it's hard for it to detect barrels that are close to each other. It sometimes mistake people for barrels. 

To get rid of these problems, I plan to implement a more robust version of the algorithm using Neural Network. Basically I will be treating pointcloud as a image and feed it to a CNN. I will first transform the pointcloud to a data format that is similar to a image. Instead using color, I will be using distance as the value of each pixel.

## Requirements
Given an input pointcloud:
1. Able to recognize the position of the barrels inside the image. 
2. Able to be more accurate and more reliable than the Clustering and Ransac sollution. 

## Design
A node will be created that subscribes to 'nonground' topic. Then the pointcloud data will be tranformed into a 2D list. After that, it will be feeded to a CNN for feature detection.

 
