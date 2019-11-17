#include <ros/ros.h>
#include <parameter_assertions/assertions.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "GroundSegmenter.h"

ros::Publisher g_ground_pub;
ros::Publisher g_nonground_pub;
ros::Publisher g_marker_pub;
ros::Subscriber g_velodyne_sub;
std::string g_ground_topic;
std::string g_nonground_topic;
std::string g_velodyne_topic;
int g_num_segments;
double g_error_t;
double g_slope_t;
double g_intercept_z_t;
double g_dist_t;

void debugViz(GroundSegmenter segmenter)
{

  visualization_msgs::MarkerArray lines;
  int i = 0;

  for (const auto &seg : segmenter.segments)
  {
    for (Line line : seg.second.lines) 
    {
      for (Prototype pt : line.model_points) 
      {
        visualization_msgs::Marker points;
        points.header.frame_id = "/lidar";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = i++;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.b = pt.blue;
        points.color.r = pt.red;
        points.color.g = pt.green;
        points.color.a = 1.0;
        geometry_msgs::Point p;
        p.x = pt.point.x;
        p.y = pt.point.y;
        p.z = pt.point.z;
        points.points.push_back(p);
        lines.markers.push_back(points);
      }
      visualization_msgs::Marker line_list;
      line_list.header.frame_id = "/lidar";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "lines";
      line_list.id = i++;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.scale.x = 0.025;
      line_list.color.b = line.blue;
      line_list.color.r = line.red;
      line_list.color.g = line.green;
      line_list.color.a = 1.0;
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
      lines.markers.push_back(line_list);
    }
  }
  g_marker_pub.publish(lines);
}

void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
{ 
  GroundSegmenter segmenter = GroundSegmenter(msg->points, g_num_segments, g_slope_t, g_intercept_z_t, g_dist_t);
  segmenter.processSegments();
  segmenter.getLinesFromSegments(g_error_t);
  debugViz(segmenter);
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points; 
  ground_points.header.frame_id = "/lidar";
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> nonground_points;
  nonground_points.header.frame_id = "/lidar";
  segmenter.classifyPoints(ground_points, nonground_points);
  g_ground_pub.publish(ground_points);
  g_nonground_pub.publish(nonground_points);
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
  assertions::getParam(pNh, "error_t", g_error_t);
  assertions::getParam(pNh, "slope_t", g_slope_t); 
  assertions::getParam(pNh, "intercept_z_t", g_intercept_z_t);
  assertions::getParam(pNh, "dist_t", g_dist_t);
  
  g_ground_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(g_ground_topic, 1);
  g_nonground_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(g_nonground_topic, 1);
  g_marker_pub = n.advertise<visualization_msgs::MarkerArray>("Lines_array", 1);
  g_velodyne_sub = n.subscribe(g_velodyne_topic, 1, velodyneCallback);

  ros::spin();

}