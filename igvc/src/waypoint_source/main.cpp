#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <igvc/StringUtils.hpp>
#include <gps_common/conversions.h>
#include <string>

using namespace std;
using namespace geometry_msgs;

ros::Publisher waypoint_pub;
vector<PointStamped> waypoints;
PointStamped current_waypoint;

void loadWaypointsFile(string path, vector<PointStamped>& waypoints)
{
    if(path.empty())
    {
        ROS_ERROR_STREAM("Could not load empty file path.");
        return;
    }

    ifstream file;
    file.open(path.c_str());

    if(!file.is_open())
    {
        ROS_INFO_STREAM("Could not open file: " << path);
        return;
    }

    string line = "";
    auto lineIndex = 1;
    while(!file.eof())
    {
        getline(file, line);

        if(!line.empty())
        {
            vector<string> tokens = split(line, ',');

            if(tokens.size() != 2)
            {
                ROS_ERROR_STREAM(path << ":" << lineIndex << " - " << tokens.size() << " tokens instead of 2.");
                return;
            }

            auto lat = stof(tokens[0]);
            auto lon = stof(tokens[1]);

            PointStamped p;

            gps_common::UTM(lat, lon, &(p.point.x), &(p.point.y));

            waypoints.push_back(p);
        }

        lineIndex++;
    }
}

double distanceBetweenPoints(const Point &p1, const Point &p2)
{
    return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
}

void positionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    if(distanceBetweenPoints(msg->pose.pose.position, current_waypoint.point) < 1.0)
    {
        auto seq = current_waypoint.header.seq + 1;
        current_waypoint = waypoints.front();
        waypoints.erase(waypoints.begin());
        current_waypoint.header.stamp = ros::Time::now();
        current_waypoint.header.seq = seq;
        current_waypoint.header.frame_id = "base_footprint";
        waypoint_pub.publish(current_waypoint);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_source");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    string path;
    nhp.getParam("file", path);

    loadWaypointsFile("", waypoints);

    if(!waypoints.empty())
    {
        current_waypoint = waypoints[0];
        waypoints.erase(waypoints.begin());
        waypoint_pub.publish(current_waypoint);

        nh.advertise<PointStamped>("/waypoint", 1);

        nh.subscribe("/robot_pose_ekf/odom_combined", 1, positionCallback);

        ros::spin();
    }

    return 0;
}
