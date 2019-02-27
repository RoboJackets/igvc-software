#include <geometry_msgs/PointStamped.h>
#include <igvc_utils/NodeUtils.hpp>
#include <ros/ros.h>
#include <string>

// the filtered waypoint to publish
geometry_msgs::PointStamped filtered_waypoint;
// maximum deviation between two consecutive waypoints
// before the filtered waypoint is updated
double distance_threshold;

// initial waypoint received
bool waypoint_recieved = false;

void waypointUnfilteredCallback(const geometry_msgs::PointStamped& msg)
{

    // filtered and new waypoint are further apart than the threshold, update
    // the filtered waypoint
    if (!waypoint_recieved)
    {
        // first waypoint received
        waypoint_recieved = true;

        filtered_waypoint.point.x = msg.point.x;
        filtered_waypoint.point.y = msg.point.y;
    }
    else if (igvc::get_distance(filtered_waypoint.point.x, filtered_waypoint.point.y,
                            msg.point.x, msg.point.y) > distance_threshold)
    {
        filtered_waypoint.point.x = msg.point.x;
        filtered_waypoint.point.y = msg.point.y;
    }

    // update timestamp and sequence #
    filtered_waypoint.header.stamp = ros::Time::now();
    filtered_waypoint.header.seq++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_filter");

    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    std::string input_topic;
    igvc::getParam(pNh, "input_topic", input_topic);
    igvc::getParam(pNh, "distance_threshold", distance_threshold);

    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PointStamped>("/waypoint", 1);
    ros::Subscriber waypoint_unfiltered_sub = nh.subscribe(input_topic, 1, waypointUnfilteredCallback);

    filtered_waypoint.header.frame_id = "odom";
    filtered_waypoint.header.seq = 0;

    ros::Rate rate(1);

    while(ros::ok())
    {
        ros::spinOnce();  // handle subscriber callbacks

        // wait for first waypoint to be recieved before publishing
        if (!waypoint_recieved)
        {
            rate.sleep();
            continue;
        }

        auto waypoint_for_pub = filtered_waypoint;
        waypoint_pub.publish(waypoint_for_pub);

        rate.sleep();
    }

    return 0;
}
