#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

class TestPathPlanner : public testing::Test
{
public:
  TestPathPlanner()
    : handle()
    , mock_map_pub(handle.advertise<igvc_msgs::map>("/map", 1))
    , mock_waypoint_pub(handle.advertise<geometry_msgs::PointStamped>("/waypoint", 1))
    , expanded_sub(handle.subscribe("/expanded", 1, &TestPathPlanner::expandedCallback, this))
    , path_sub(handle.subscribe("/path", 1, &TestPathPlanner::pathCallback, this))
  {
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
  }

  void expandedCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
  }

protected:
  virtual void SetUp()
  {
    while (!IsNodeReady())
    {
      ros::spinOnce();
    }
  }

  virtual void TearDown()
  {
  }

  bool IsNodeReady()
  {
    return (mock_map_pub.getNumSubscribers() > 0) && (mock_waypoint_pub.getNumSubscribers() > 0)
      && (expanded_sub.getNumPublishers() > 0) && (path_sub.getNumPublishers() > 0);
  }

  ros::NodeHandle handle;
  ros::Publisher mock_map_pub;
  ros::Publisher mock_waypoint_pub;
  ros::Subscriber expanded_sub;
  ros::Subscriber path_sub;
};

TEST_F(TestPathPlanner, EmptyMap)
{
  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(10, 10, CV_8UC1, 0.0);
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.resolution = 0.1;
  map.x_position = 0.0;
  map.y_position = 0.0;
  map.theta = M_PI / 4;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 9;
  waypoint.point.y = 9;
  mock_waypoint_pub.publish(waypoint);

  const nav_msgs::Path::ConstPtr& response = ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  EXPECT_TRUE(response.get() != nullptr);
  EXPECT_EQ(response->poses.size(), 7);
  EXPECT_EQ(response->poses.size(), 7);
  for(double i = 0; i < 7; i++) {
    EXPECT_EQ(i, response->poses[i].pose.position.x);
    EXPECT_EQ(i, response->poses[i].pose.position.y);
  }
}

TEST_F(TestPathPlanner, CheckGoalThreshold)
{

  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(10, 10, CV_8UC1, 0.0);
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.x_position = 0.0;
  map.y_position = 0.0;
  map.theta = M_PI / 4;
  map.resolution = 0.1;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 9;
  waypoint.point.y = 0;
  mock_waypoint_pub.publish(waypoint);

  const nav_msgs::Path::ConstPtr& response = ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  EXPECT_TRUE(response.get() != nullptr);
  EXPECT_EQ(response->poses.size(), 6);
  for(double i = 0; i < 6; i++) {
    EXPECT_EQ(i, response->poses[i].pose.position.x);
    EXPECT_EQ(0, response->poses[i].pose.position.y);
  }
}

TEST_F(TestPathPlanner, AvoidsObstacle)
{
  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(10, 10, CV_8UC1, 0.0);
  map_image.at<uchar>(3,3) = 255;
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.x_position = 0.0;
  map.y_position = 0.0;
  map.theta = M_PI / 4;
  map.resolution = 0.1;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 9;
  waypoint.point.y = 9;
  mock_waypoint_pub.publish(waypoint);

  const nav_msgs::Path::ConstPtr& response = ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  EXPECT_TRUE(response.get() != nullptr);
  EXPECT_EQ(response->poses.size(), 11);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pathplanner");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
>>>>>>> pathplanner
}
