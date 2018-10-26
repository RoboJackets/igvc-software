#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/mat.hpp>

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
    return (mock_map_pub.getNumSubscribers() > 0) && (mock_waypoint_pub.getNumSubscribers() > 0) &&
           (expanded_sub.getNumPublishers() > 0) && (path_sub.getNumPublishers() > 0);
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
  map.x = 0.0;
  map.y = 0.0;
  map.x_initial = 0.0;
  map.y_initial = 0.0;
  map.orientation = M_PI / 4;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 1.0;
  waypoint.point.y = 1.0;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 8);
  for (double i = 0; i < 8; i++)
  {
    EXPECT_NEAR(i / 10, response->poses[i].pose.position.x, 0.001);
    EXPECT_NEAR(i / 10, response->poses[i].pose.position.y, 0.001);
  }
}

TEST_F(TestPathPlanner, TestDifferentStartLocation)
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
  map.x = 3;
  map.y = 3;
  map.x_initial = 0;
  map.y_initial = 0;
  map.orientation = M_PI / 4;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 1.0;
  waypoint.point.y = 1.0;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 5);
  for (double i = 3; i < 8; i++)
  {
    EXPECT_NEAR(i / 10, response->poses[i - 3].pose.position.x, 0.001);
    EXPECT_NEAR(i / 10, response->poses[i - 3].pose.position.y, 0.001);
  }
}

TEST_F(TestPathPlanner, TestInitialLocationOffset)
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
  map.x = 4;
  map.y = 4;
  map.x_initial = 1;
  map.y_initial = 1;
  map.orientation = M_PI / 4;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 1.0;
  waypoint.point.y = 1.0;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 5);
  for (double i = 3; i < 8; i++)
  {
    EXPECT_NEAR(i / 10, response->poses[i - 3].pose.position.x, 0.001);
    EXPECT_NEAR(i / 10, response->poses[i - 3].pose.position.y, 0.001);
  }
}

TEST_F(TestPathPlanner, AvoidsObstacle)
{
  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(10, 10, CV_8UC1, 0.0);
  map_image.at<uchar>(3, 4) = 255;
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.x = 0.0;
  map.y = 0.0;
  map.x_initial = 0.0;
  map.y_initial = 0.0;
  map.orientation = M_PI / 4;
  map.resolution = 0.1;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 1.0;
  waypoint.point.y = 1.0;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));

  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 11);

  EXPECT_NEAR(response->poses[0].pose.position.x, 0, 0.01);
  EXPECT_NEAR(response->poses[0].pose.position.y, 0, 0.01);

  EXPECT_NEAR(response->poses[1].pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(response->poses[1].pose.position.y, 0.1, 0.01);

  EXPECT_NEAR(response->poses[2].pose.position.x, 0.2, 0.01);
  EXPECT_NEAR(response->poses[2].pose.position.y, 0.1, 0.01);

  EXPECT_NEAR(response->poses[3].pose.position.x, 0.3, 0.01);
  EXPECT_NEAR(response->poses[3].pose.position.y, 0.1, 0.01);

  EXPECT_NEAR(response->poses[4].pose.position.x, 0.4, 0.01);
  EXPECT_NEAR(response->poses[4].pose.position.y, 0.1, 0.01);

  EXPECT_NEAR(response->poses[5].pose.position.x, 0.5, 0.01);
  EXPECT_NEAR(response->poses[5].pose.position.y, 0.1, 0.01);

  EXPECT_NEAR(response->poses[6].pose.position.x, 0.6, 0.01);
  EXPECT_NEAR(response->poses[6].pose.position.y, 0.2, 0.01);

  EXPECT_NEAR(response->poses[7].pose.position.x, 0.7, 0.01);
  EXPECT_NEAR(response->poses[7].pose.position.y, 0.3, 0.01);

  EXPECT_NEAR(response->poses[8].pose.position.x, 0.7, 0.01);
  EXPECT_NEAR(response->poses[8].pose.position.y, 0.4, 0.01);

  EXPECT_NEAR(response->poses[9].pose.position.x, 0.7, 0.01);
  EXPECT_NEAR(response->poses[9].pose.position.y, 0.5, 0.01);

  EXPECT_NEAR(response->poses[10].pose.position.x, 0.8, 0.01);
  EXPECT_NEAR(response->poses[10].pose.position.y, 0.6, 0.01);
}

TEST_F(TestPathPlanner, EmptyMapHorizontal)
{
  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(50, 50, CV_8UC1, 0.0);
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.resolution = 0.2;
  map.x = 5;
  map.y = 0;
  map.x_initial = 0;
  map.y_initial = 0;
  map.orientation = M_PI / 2;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = 1.0;
  waypoint.point.y = 2.0;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 9);
  for (double i = 0; i < 9; i++)
  {
    EXPECT_NEAR(1, response->poses[i].pose.position.x, 0.001);
    EXPECT_NEAR(i / 5, response->poses[i].pose.position.y, 0.001);
  }
}

TEST_F(TestPathPlanner, EmptyMapLarge)
{
  igvc_msgs::map map;
  cv::Mat map_image = cv::Mat(500, 500, CV_8UC1, 0.0);
  map.header.stamp = ros::Time::now();

  std_msgs::Header image_header;
  image_header.stamp = ros::Time::now();

  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::MONO8, map_image);
  sensor_msgs::Image image_message;
  image_bridge.toImageMsg(image_message);
  map.image = image_message;

  map.resolution = 0.2;
  map.x = 250;
  map.y = 250;
  map.x_initial = 250;
  map.y_initial = 250;
  map.orientation = M_PI / 2;

  mock_map_pub.publish(map);

  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = -0.278;
  waypoint.point.y = 11.08;
  mock_waypoint_pub.publish(waypoint);

  sleep(1);

  const nav_msgs::Path::ConstPtr& response =
      ros::topic::waitForMessage<nav_msgs::Path>(path_sub.getTopic(), ros::Duration(5));
  ASSERT_TRUE(response.get() != nullptr);
  ASSERT_EQ(response->poses.size(), 54);
  EXPECT_NEAR(0, response->poses[0].pose.position.x, 0.001);
  EXPECT_NEAR(0, response->poses[0].pose.position.y, 0.001);
  for (double i = 1; i < 10; i++)
  {
    EXPECT_NEAR(0, response->poses[i].pose.position.x, 0.001);
    EXPECT_NEAR(i / 5, response->poses[i].pose.position.y, 0.001);
  }
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
}
