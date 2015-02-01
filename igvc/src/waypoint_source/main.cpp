#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <igvc/StringUtils.hpp>
#include <igvc/GPSUtils.h>

using namespace std;
using namespace sensor_msgs;

ros::Publisher waypoint_pub;
std::vector<NavSatFix> waypoints;
NavSatFix current_waypoint;

void loadWaypointsFile(string path, vector<NavSatFix>& waypoints)
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

            NavSatFix data;
            data.latitude = atof(tokens[0].c_str());
            data.longitude = atof(tokens[1].c_str());

            waypoints.push_back(data);
        }

        lineIndex++;
    }
}

void fixCallback(const NavSatFixConstPtr& msg)
{
    if(GPSUtils::coordsToMeter(msg->latitude, msg->longitude, current_waypoint.latitude, current_waypoint.longitude) < 1.0)
    {
        current_waypoint = waypoints[0];
        waypoints.erase(waypoints.begin());
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

        nh.advertise<NavSatFix>("/waypoint", 1);

        nh.subscribe("/fix", 1, fixCallback);

        ros::spin();
    }

    return 0;
}
