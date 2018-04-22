#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h> // to check transforms
#include <opencv2/opencv.hpp> // for cv::Mat
#include <opencv2/core/eigen.hpp> // for cv to eigen ish
#include <cv_bridge/cv_bridge.h> // for eigen to cv ish

// messages for mapper
//   /scan/pointcloud
//   /usb_cam_center/line_cloud
//   /pothole_cloud


class TestNewMapper : public testing::Test
{
public:
  TestNewMapper()
    : handle()
    , mock_map_pub(handle.advertise<sensor_msgs::Joy>("/map", 1))
    , lidar_sub(handle.subscribe("/scan/pointcloud", 1, &TestNewMapper::lidarCallback, this))
    , cam_sub(handle.subscribe("/usb_cam_center/line_cloud", 1, &TestNewMapper::camCallBack, this))
    , pothole_sub(handle.subscribe("/pothole_cloud", 1, &TestNewMapper::potholeCallback, this))
  {
  }

  void lidarCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
  {
  }
  void camCallBack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
  {
  }
  void potholeCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
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
    return (mock_map_pub.getNumSubscribers() > 0) &&
     (lidar_sub.getNumPublishers() > 0) &&
     (cam_sub.getNumPublishers() > 0) &&
     (pothole_sub.getNumPublishers() > 0)
  }

  ros::NodeHandle handle;
  ros::Publisher mock_joy_pub;
  ros::Subscriber motor_sub;
};

TEST_Lidar(TestNewMapper, FullForward)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { 0, 1.0, 0, 1.0 };
  joy_msg.buttons = { 0, 0, 0, 0 };
  mock_joy_pub.publish(joy_msg);

  const igvc_msgs::velocity_pair::ConstPtr& response =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motor_sub.getTopic(), ros::Duration(1));

  EXPECT_TRUE(response.get() != nullptr);
  EXPECT_EQ(response->left_velocity, 1.0);
  EXPECT_EQ(response->right_velocity, 1.0);
}

