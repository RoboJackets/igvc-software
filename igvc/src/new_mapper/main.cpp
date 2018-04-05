#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

ros::Publisher pointcloud_pub;
cv::Mat published_map; // get to work
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> global_map;
double resolution;
double position [2] = {0, 0};
int length;
int width;

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  //transform pointcloud into the occupancy grid, no filtering right now
  bool offMap = false;
  //for(pcl::PointXYZRGB point : msg) // how to iterate through all points??
  //cv::Mat frame(msg->points);
  pcl::PointCloud<pcl::PointXYZ>::const_iterator point;
  for (point = msg->points.begin(); point < msg->points.end(); point++)
  {
    //assuming is meters from robot origin
    //transform coordinates
    double x = point->x / resolution;
    double y = point->y / resolution;

    x += position[0];
    y += position[1];
    if(x > 0 && y > 0 && x < length && y < width)
    {
      global_map(x, y) = 1.0;
    } else if(!offMap){
      ROS_WARN_STREAM("Some points out of range, won't be put on map.");
      offMap = true;
    }
  }


  pointcloud_pub.publish(published_map);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "new_mapper");
  ros::NodeHandle nh;

  std::string topics;
  std::list<ros::Subscriber> subs;

  ros::NodeHandle pNh("~");

  double start_x;
  double start_y;

  pNh.getParam("topics", topics);
  pNh.getParam("occupancy_grid_length", length);
  pNh.getParam("occupancy_grid_width", width);
  pNh.getParam("qoccupancy_grid_resolution", resolution);
  pNh.getParam("start_X", start_x);
  pNh.getParam("start_Y", start_y);

  length = (int) (length / resolution);
  width = (int) (width / resolution);
  position[0] = start_x / resolution;
  position[1] = start_y / resolution;
  //global_map = Eigen::Matrix<float, Dynamic, Dynamic>(length, width);

  published_map(length, width, 2.0, cv::Scalar(0,0,0)); // I cant instatiate this
  //https://docs.opencv.org/2.4/doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.html
  global_map(published_map.data()); // i can't instantiate this either
  //https://stackoverflow.com/questions/14783329/opencv-cvmat-and-eigenmatrix

  //will need to change when subscribe to multiple topics
  std::list<std::string> tokens = {topics};
  for (std:: string t : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << t);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(t, 1,boost::bind(frame_callback, _1, t)));
  }
  pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map", 1);

  ros::spin();
}
