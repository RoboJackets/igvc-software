#include "elevation_map_height_node.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <parameter_assertions/assertions.h>
#include <string>

ElevationMapHeightNode::ElevationMapHeightNode(){
    private_nh_ = ros::NodeHandle("~");
    std::string output_topic;
    assertions::getParam(private_nh_, "output_topic", output_topic);
    robot_pose_estimate_pub_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_topic, 1);
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
    geometry_msgs::PoseWithCovarianceStamped new_pose;
    new_pose.header = robot_pose_.header;
    double height = map.get("elevation_smooth")(index[0], index[1]);
    new_pose.pose.pose.position = robot_pose_.pose.pose.position;
    new_pose.pose.pose.position.z = height;
    new_pose.pose.covariance = robot_pose_.pose.covariance;
    new_pose.pose.covariance[14] = map.get("variance")(index[0], index[1]);
    robot_pose_estimate_pub_.publish(new_pose);
}

void ElevationMapHeightNode::robotPoseCallback(const nav_msgs::Odometry& robot_pose) {
    robot_pose_.header = robot_pose.header;
    robot_pose_.pose = robot_pose.pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elevation_map_height_node");
    ElevationMapHeightNode emhn = ElevationMapHeightNode();
    ros::spin();
}
