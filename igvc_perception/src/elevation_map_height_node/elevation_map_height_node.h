#ifndef SRC_ELEVATIONMAPHEIGHTNODE_H
#define SRC_ELEVATIONMAPHEIGHTNODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>

class ElevationMapHeightNode {
public:
    ElevationMapHeightNode();

private:
    ros::NodeHandle private_nh_;
    ros::Publisher robot_pose_estimate_pub_;
    ros::Subscriber elevation_map_sub_;
    ros::Subscriber robot_pose_sub_;

    geometry_msgs::PoseWithCovarianceStamped robot_pose_;

    void elevationMapCallback(const grid_map_msgs::GridMap& elevation_map);
    void robotPoseCallback(const nav_msgs::Odometry& robot_pose);
};


#endif //SRC_ELEVATIONMAPHEIGHTNODE_H
