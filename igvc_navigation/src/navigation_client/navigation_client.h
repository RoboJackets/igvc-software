#ifndef SRC_NAVIGATION_CLIENT_H
#define SRC_NAVIGATION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

using MoveBaseClient = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;

class NavigationClient
{
public:
    NavigationClient();
    ros::NodeHandle nh_;
    ros::Publisher waypoints_pub;
    ros::Publisher current_waypoint_pub;
    ros::Subscriber rviz_sub_;
    ros::Subscriber odom_sub_;
    tf::TransformListener tf_listener_;

private:
    // action lib client
    MoveBaseClient client = MoveBaseClient("move_base_flex/move_base", true);

    // waypoint poses to send
    std::vector<geometry_msgs::PointStamped> waypoints_queue_;

    // params
    bool reading_from_file_; // true if reading from waypoint file, false if from rviz
    std::string waypoint_file_path_;       // path to waypoint file
    double waypoint_radius_; // detection radius for waypoint

    /**
    Loads waypoints from path_, transforms them into the UTM frame, and stores
    them in waypoints_.
    */
    void load_waypoints_file();

    /**
    degrees minutes seconds to decimal degrees conversion function

    @param[in] dms lat or long in degrees minutes seconds
    @return input value in decimal degrees
    */
    static double dms_to_dec(std::string dms);

    /**
    sends the PoseStamped as a goal

    @param[in] PoseStamped to send
     */
    void sendPoseAsGoal(const geometry_msgs::PoseStamped&);

    /**
    sends the PoseStamped as a goal and waits

    @param[in] PoseStamped to send
     */
    void sendPoseAsGoalAndWait(const geometry_msgs::PoseStamped&);

    /**
    sends the PointStamped as a goal, orientation is that of yaw = 0

    @param[in] PointStamped to send
     */
    void sendPointAsGoal(const geometry_msgs::PointStamped&);

    /**
    sends the PointStamped as a goal and waits, orientation is that of yaw = 0

    @param[in] PointStamped to send
     */
    void sendPointAsGoalAndWait(const geometry_msgs::PointStamped&);

    /**
    (ROS Callback) Checks if the robot is less than a threshold distance from the
    current waypoint using the robot's current odometry.

    @param[in] msg most recent odometry message on the "odometry/filtered" topic
    */
    void position_callback(const nav_msgs::OdometryConstPtr& msg);

    /**
    (ROS Callback) sends waypoints from rviz to the navigation server

    @param[in]
    */
    void rvizWaypointCallback(const geometry_msgs::PoseStamped&);
};


#endif //SRC_NAVIGATION_CLIENT_H
