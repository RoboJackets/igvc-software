# Barrel Detection Research

**Author: Joshua Viszlai**


Paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5548059&tag=1

## Purpose of Paper
This paper addresses segmentation of a 3d point cloud into first, ground points v. non-ground points, and second, segmentation of non-gruond points into separate obstacles. For our uses currently, we can focus on their implementation of segmenting ground points v. non-ground points, and treat non-ground points as barrels.

## Summary of Paper
The general idea presented for segmenting out the ground plane consists of fitting a 3d pointcloud into many, small lines and then evaluating them against the following conditions:
* The slope *m* of the fitted line must be below a specified threshold *T<sub>m</sub>*, i.e the line should be close to flat
* The y-intercept, *b*, must be below a specified threshold *T<sub>b</sub>* i.e tops of barrels/plateaus are not ground
* The root mean square error of the fit of the points to the line must be below a specified threshold *T<sub>RMSE</sub>* i.e we can't effectively evaluate points if the line isn't representative
* The distance of the first point of a line to the previously fitted line must be below a specified threshold *T<sub>d<sub>prev</sub></sub>* i.e lines should have smooth transitions between eachother

Meeting these conditions classifies the points in the line as ground points. Points in other lines are therefore classified as non-ground points. 

Then, the process of fitting 3d points into these lines is described as:
1. Partition the points into segments that are slices of an infinitely large pie with the robot being the center
2. Sort points into bins within these segments, with boundaries defined by radii from the center (the robot)
3. If there is more than one point in the same bin, keep the one with the smallest height
4. Create a set of points per segment, consisting of the points in each bin of that segment
5. Using this set of points, extract multiple lines by incrementally: creating a line model from two points, trying to add subsequent points to this model, splitting off and starting a new line model if subsequent points don't fit well. (described more in-depth here: https://link.springer.com/content/pdf/10.1007%2Fs10514-007-9034-y.pdf)

These lines are then classified as ground v. non-ground, and any points not apart of a line model are mapped to the closest line within a distance threshold *T<sub>d<sub>ground</sub></sub>*, outside of the threshold, points are classified as non-ground.