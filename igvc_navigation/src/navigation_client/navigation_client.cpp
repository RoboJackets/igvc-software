#include <parameter_assertions/assertions.h>
#include "navigation_client.h"

#include <igvc_utils/StringUtils.hpp>
#include <robot_localization/navsat_conversions.h>
#include <igvc_utils/NodeUtils.hpp>

NavigationClient::NavigationClient() {
    ros::NodeHandle pNh("~");

    assertions::getParam(pNh, "reading_from_file", reading_from_file_);
    assertions::getParam(pNh, "path", path_);
    assertions::getParam(pNh, "waypoint_radius", waypoint_radius_);

    rviz_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavigationClient::rvizWaypointCallback, this);

    // wait for the utm->odom transform to become available
    while (!tf_listener_.waitForTransform("odom", "utm", ros::Time(0), ros::Duration(5.0)))
    {
        ROS_INFO_STREAM("utm->odom transform not found. waiting...");
    }

    ROS_INFO_STREAM("utm->odom transform found!");

    if(reading_from_file_){
        odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &NavigationClient::position_callback, this);
        load_waypoints_file();
        sendPointAsGoal(waypoints_queue_.front());

        // transform the waypoint at the head of the waypoints_ vector from UTM to odom
        tf_listener_.transformPoint("odom", ros::Time(0), waypoints_queue_.front(), "odom", current_waypoint_odom_);
    }
    ros::spin();
}

void NavigationClient::load_waypoints_file() {
    ROS_INFO_STREAM_ONCE("Loading waypoints from " << path_);

    if (path_.empty())
    {
        ROS_ERROR_STREAM("Could not load waypoints. Empty file path.");
        return;
    }

    std::ifstream file;
    file.open(path_.c_str());

    if (!file.is_open())
    {
        ROS_INFO_STREAM("Could not open file: " << path_);
        return;
    }

    std::string line;
    auto lineIndex = 1;
    while (!file.eof())
    {
        getline(file, line);

        if (!line.empty() && line[0] != '#')
        {
            std::vector<std::string> tokens = split(line, ',');

            if (tokens.size() != 2)
            {
                ROS_ERROR_STREAM(path_ << ":" << lineIndex << " - " << tokens.size() << " tokens instead of 2.");
                return;
            }

            // convert latitude and longiture to decimal degrees if currently in degrees minutes second. specified by '?'
            // symbol
            double lat = (tokens[0].find('?') != std::string::npos) ? dms_to_dec(tokens[0]) : stod(tokens[0]);
            double lon = (tokens[1].find('?') != std::string::npos) ? dms_to_dec(tokens[1]) : stod(tokens[1]);

            // transform latitude and longitude to UTM frame
            geometry_msgs::PointStamped p;
            RobotLocalization::NavsatConversions::UTM(lat, lon, &(p.point.x), &(p.point.y));
            p.header.frame_id = "utm";
            waypoints_queue_.push_back(p);
        }
        lineIndex++;
    }

    ROS_INFO_STREAM_ONCE(waypoints_queue_.size() << " waypoints loaded.");
}

double NavigationClient::dms_to_dec(std::string dms) {
    auto qMarkIter = dms.find('?');
    auto aposIter = dms.find('\'');
    auto qouteIter = dms.find('\"');
    auto degrees = stod(dms.substr(0, qMarkIter));
    auto minutes = stod(dms.substr(qMarkIter + 1, aposIter));
    auto seconds = stod(dms.substr(aposIter + 1, qouteIter));
    auto dirChar = dms[dms.size() - 1];

    degrees += minutes / 60.0;
    degrees += seconds / 3600.0;

    if (dirChar == 'W' || dirChar == 'S')
        degrees *= -1;

    return degrees;
}

void NavigationClient::sendPoseAsGoal(const geometry_msgs::PoseStamped& pose) {
    mbf_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    client.sendGoal(goal);
}

void NavigationClient::sendPoseAsGoalAndWait(const geometry_msgs::PoseStamped& pose) {
    mbf_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    client.sendGoalAndWait(goal);
}

void NavigationClient::sendPointAsGoal(const geometry_msgs::PointStamped& point) {
    mbf_msgs::MoveBaseGoal goal;

    goal.target_pose.header = point.header;
    goal.target_pose.pose.position = point.point;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    client.sendGoal(goal);
}

void NavigationClient::sendPointAsGoalAndWait(const geometry_msgs::PointStamped& point) {
    mbf_msgs::MoveBaseGoal goal;

    goal.target_pose.header = point.header;
    goal.target_pose.pose.position = point.point;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    client.sendGoalAndWait(goal);
}

void NavigationClient::position_callback(const nav_msgs::OdometryConstPtr& msg) {
    if (igvc::get_distance(msg->pose.pose.position, current_waypoint_odom_.point) < waypoint_radius_){
        // sets the orientation to be the angle the robot enters the radius from
        geometry_msgs::PoseStamped pose;
        pose.header = waypoints_queue_.front().header;
        pose.pose.position = waypoints_queue_.front().point;
        double angle = std::atan2(current_waypoint_odom_.point.y - msg->pose.pose.position.y, current_waypoint_odom_.point.x - msg->pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        // sends pose to be next goal
        sendPoseAsGoalAndWait(pose);

        // move on to next waypoint
        waypoints_queue_.erase(waypoints_queue_.begin());
        if(waypoints_queue_.size() > 1){
            sendPointAsGoal(waypoints_queue_.front());
            ROS_INFO_STREAM("");
            // transform the waypoint at the head of the waypoints_ vector from UTM to odom
            tf_listener_.transformPoint("odom", ros::Time(0), waypoints_queue_.front(), "odom", current_waypoint_odom_);
        } else {

        }
    }
}

void NavigationClient::rvizWaypointCallback(const geometry_msgs::PoseStamped& pose) {
    if(reading_from_file_){
        ROS_ERROR("Cannot send goal from rviz, reading from file.");
    } else {
        sendPoseAsGoal(pose);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_client");
    NavigationClient();
}