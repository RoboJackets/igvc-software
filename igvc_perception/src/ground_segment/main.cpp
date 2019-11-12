#include <ros/ros.h>
#include <parameter_assertions/assertions.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include "GroundSegmenter.h"

ros::Publisher g_ground_pub;
ros::Publisher g_nonground_pub;
ros::Publisher g_marker_pub;
ros::Publisher g_marker2_pub;
ros::Subscriber g_velodyne_sub;
std::string g_ground_topic;
std::string g_nonground_topic;
std::string g_velodyne_topic;
int g_num_segments;
double g_bin_width;
double g_error_t;

void debugViz(GroundSegmenter segmenter)
{
  visualization_msgs::Marker points;
  points.header.frame_id = "/lidar";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.color.g = 1.0;
  points.color.a = 1.0;
  for (const auto &seg : segmenter.segments)
  {
    for (Prototype pt : seg.second.prototype_points) 
    {
      geometry_msgs::Point p;
      p.x = pt.point.x;
      p.y = pt.point.y;
      p.z = pt.point.z;
      points.points.push_back(p);
    }
  }
  g_marker_pub.publish(points);
  
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/lidar";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.025;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  for (const auto &seg : segmenter.segments)
  {
    for (Line line : seg.second.lines) 
    {
      geometry_msgs::Point p1;
      p1.x = line.intercept(0, 0) - 1 * line.params(0, 0);
      p1.y = line.intercept(1, 0) - 1 * line.params(1, 0);
      p1.z = line.intercept(2, 0) - 1 * line.params(2, 0);
      line_list.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = line.intercept(0, 0) + 1 * line.params(0, 0);
      p2.y = line.intercept(1, 0) + 1 * line.params(1, 0);
      p2.z = line.intercept(2, 0) + 1 * line.params(2, 0);
      line_list.points.push_back(p2);
    }
  }
  g_marker2_pub.publish(line_list);
}

void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
{ 
  GroundSegmenter segmenter = GroundSegmenter(msg->points, g_num_segments, g_bin_width);
  segmenter.processSegments();
  segmenter.getLinesFromSegments(g_error_t);
  debugViz(segmenter);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_segment");

  ros::NodeHandle n;
  ros::NodeHandle pNh("~");

  assertions::getParam(pNh, "ground_topic", g_ground_topic);
  assertions::getParam(pNh, "nonground_topic", g_nonground_topic);
  assertions::getParam(pNh, "velodyne_topic", g_velodyne_topic);
  assertions::getParam(pNh, "num_segments", g_num_segments);
  assertions::getParam(pNh, "bin_width", g_bin_width);
  assertions::getParam(pNh, "error_t", g_error_t);

  g_ground_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(g_ground_topic, 1);
  g_nonground_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(g_nonground_topic, 1);
  g_marker_pub = n.advertise<visualization_msgs::Marker>("Segment_Points", 1);
  g_marker2_pub = n.advertise<visualization_msgs::Marker>("Lines", 1);
  g_velodyne_sub = n.subscribe(g_velodyne_topic, 1, velodyneCallback);

  ros::spin();

}