#include "elevation_map_height_node.h"
#include <grid_map_ros/GridMapRosConverter.hpp>

ElevationMapHeightNode::ElevationMapHeightNode(){
    private_nh_ = ros::NodeHandle("~");
    robot_pose_estimate_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("/elevation_map/robot_pose", 1);
    elevation_map_sub_ = private_nh_.subscribe("/slope/gridmap", 1, &ElevationMapHeightNode::elevationMapCallback, this);
    robot_pose_sub_ = private_nh_.subscribe("/odometry/filtered", 1, &ElevationMapHeightNode::robotPoseCallback, this);
}

void ElevationMapHeightNode::elevationMapCallback(const grid_map_msgs::GridMap &elevation_map) {
    // get map from message
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(elevation_map, map);
    // get index corresponding to robot position
    grid_map::Index index;
    grid_map::Position position = {robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y};
    map.getIndex(position, index);
    // create new pose message with elevation map height
    nav_msgs::Odometry new_pose;
    new_pose.header = robot_pose_.header;
    new_pose.child_frame_id = robot_pose_.child_frame_id;
    new_pose.twist = robot_pose_.twist;

    double height = map.get("elevation")(index[0], index[1]);
    new_pose.pose.pose.position = robot_pose_.pose.pose.position;
    new_pose.pose.pose.position.z = height;
    //new_pose.pose.covariance TODO
    robot_pose_estimate_pub_.publish(new_pose);
}

void ElevationMapHeightNode::robotPoseCallback(const nav_msgs::Odometry& robot_pose) {
    robot_pose_ = robot_pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elevation_map_height_node");
    ElevationMapHeightNode emhn = ElevationMapHeightNode();
    ros::spin();
}
