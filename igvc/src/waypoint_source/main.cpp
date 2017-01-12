#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <igvc/StringUtils.hpp>
#include <string>
#include <mutex>
#include "conversion.h"

using namespace std;
using namespace geometry_msgs;
using namespace ros;

ros::Publisher waypoint_pub;
vector<PointStamped> waypoints;
PointStamped current_waypoint;
Point map_origin;
mutex current_mutex;

double dmsToDec(string dms)
{
    auto qMarkIter = dms.find('?');
    auto aposIter = dms.find('\'');
    auto qouteIter = dms.find('\"');
    auto degrees = stod(dms.substr(0, qMarkIter));
    auto minutes = stod(dms.substr(qMarkIter+1, aposIter));
    auto seconds = stod(dms.substr(aposIter+1, qouteIter));
    auto dirChar = dms[dms.size() - 1];

    degrees += minutes / 60.0;
    degrees += seconds / 3600.0;

    if(dirChar == 'W' || dirChar == 'S')
        degrees *= -1;

    return degrees;
}

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

            double lat, lon;
            if(tokens[0].find('?') != string::npos)
                lat = dmsToDec(tokens[0]);
            else
                lat = stod(tokens[0]);
            if(tokens[1].find('?') != string::npos)
                lon  = dmsToDec(tokens[1]);
            else
                lon = stod(tokens[1]);

            PointStamped p;

            UTM(lat, lon, &(p.point.x), &(p.point.y));

            waypoints.push_back(p);
        }

        lineIndex++;
    }
}

double distanceBetweenPoints(const Point &p1, const Point &p2)
{
    return sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
}

void positionCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    lock_guard<mutex> lock(current_mutex);
    PointStamped cur = current_waypoint;
    cur.point.x -= map_origin.x;
    cur.point.y -= map_origin.y;

    if(distanceBetweenPoints(msg->pose.position, cur.point) < 1.0)
    {
        // advance to next waypoint.
        current_waypoint = waypoints.front();
        if(waypoints.size() > 1) {
            waypoints.erase(waypoints.begin());
            cerr << "Waypoint Source moving to next" << endl;
        }
    }
}

void originCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
    lock_guard<mutex> lock(current_mutex);
    map_origin = msg->point;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_source");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ROS_INFO_STREAM("Has param: " << nhp.hasParam("file"));

    string path;
    nhp.getParam("file", path);

    ROS_INFO_STREAM("Loading waypoints from " << path);

    waypoint_pub = nh.advertise<PointStamped>("/waypoint", 1);

    ros::Subscriber odom_sub = nh.subscribe("/odom_combined", 1, positionCallback);

    ros::Subscriber origin_sub = nh.subscribe("/map_origin", 1, originCallback);

    loadWaypointsFile(path, waypoints);

    if(!waypoints.empty())
    {
        ROS_INFO_STREAM(waypoints.size() << " waypoints found.");
        current_waypoint = waypoints.front();
        Rate rate(1); // 1 Hz
        while(ros::ok())
        {
            {
                lock_guard<mutex> lock(current_mutex);
                auto waypoint_for_pub = current_waypoint;
                waypoint_for_pub.header.stamp = ros::Time::now();
                waypoint_for_pub.header.seq++;
                waypoint_for_pub.header.frame_id = "map";
                waypoint_for_pub.point.x -= map_origin.x;
                waypoint_for_pub.point.y -= map_origin.y;
                waypoint_pub.publish(waypoint_for_pub);
            }

            ros::spinOnce();
            rate.sleep();
        }
    } else {
        ROS_ERROR("No valid waypoint entries found.");
    }

    return 0;
}
