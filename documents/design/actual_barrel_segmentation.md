# Actual barrel segmentation 

*Issue #569*

**Author:**
- Zhiyan Zhou (Frank)

## The Problem
Cluster points from non-ground pointcloud into different groups representing different objects. We can potentially use the information for things like SLAM.
End goal will be separating barrels into individual groups and extracting information such as position, Num of barrels etc.

## Proposed Solution
- Create a subscriber that suscribes to the filtered non-ground data.
- Use the clustering algorithm inside PCL.
- (If necessary)Program a clustering algorithm that take in a pointcloud and returns a clustered pointcloud.
- The way the clustering algorithm works: (http://pointclouds.org/documentation/tutorials/cluster_extraction.php) 

'''
1. create a Kd-tree representation for the input point cloud dataset P;
2. set up an empty list of clusters C, and a queue of the points that need to be checked Q;
3. then for every point pi in P, perform the following steps:
	- add pi to the current queue Q;
	- for every point pi in Q do:
		- search for the set P^i_k of point neighbors of pi in a sphere with radius r < dth;
		- for every neighbor pki in P^k_i, check if the point has already been processed, and if not add it to Q;
	when the list of all points in Q has been processed, add Q to the list of clusters C, and reset Q to an empty list
4. the algorithm terminates when all points \boldsymbol{p}_i \in P have been processed and are now part of the list of point clusters C
'''

- Visualized clustering result.
- Tune the parameter of the optimal algorithm if necessary.
- Use the clusters to calculate metrics (ex: position, num of barrels/) 
- Publish the metrics to the corresponding topic. 

## Questions & Research
- What are the common clustering algorithms?
- Any known implementations in ROS/C++?
- /igvc-software/documents/research/barrel_detection.md
- https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6338962/pdf/sensors-19-00172.pdf

## Overall Scope

### Affected Packages
- /igvc-software/igvc_perception/src/pointcloud_filter/

### Schedule

Subtask 1 (12/24/2019): Program a subscriber that subscribes pointcloud data.

Subtask 2 (2-3 week into 2019 Spring): Use the clustering algorithm inside PCL to cluster points.

Subtask 3 (/): (If necessary)Go over the paper listed: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6338962/pdf/sensors-19-00172.pdf

Subtask 4 (/): (If necessary) Implement a DBSCAN algorithm 

Subtask 5 (/): (If necessary) Test the original DBSCAN algorithm under a known distance

Subtask 6 (/): (If necessary) Work on Auto Hyperparameter Tuning. 

Subtask 7 (3-4 weeks into 2019 Spring): Test and tune the algorithm under simulated data. 
