#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <Eigen>

ros::Publisher pointcloud_pub;
tf::TransformListener *tf_listener;
Eigen::Matrix<float, Dynamic, Dynamic> global_map;
double resolution;
int[] position = [0, 0];
int length;
int width;



void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const std::string &topic)
{

}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{

}

int main(int argc, char **argv)
{
  new_map = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

  ros::init(argc, argv, "new_mapper");
  ros::NodeHandle nh;
  tf_listener = new tf::TransformListener();

  std::string topics;
  std::list<ros::Subscriber> subs;

  ros::NodeHandle pNh("~");

  int start_x;
  int start_y;

  pNh.getParam("occupancy_grid_length", length);
  pNh.getParam("occupancy_grid_width", width);
  pNh.getParam("qoccupancy_grid_resolution", resolution);
  pNh.getParam("start_X", start_x);
  pNh.getParam("start_Y", start_y);

  length = (int) (length / resolution);
  width = (int) (width / resolution);
  position[0] = (int) (start_x / resolution);
  position[1] = (int) (start_y / resolution);
  global_map = Eigen::Matrix<float, Dynamic, Dynamic>(length, width);

  ros::spin();
}
