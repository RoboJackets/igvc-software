#include <cv_bridge/cv_bridge.h>  // for eigen to cv ish
#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <pcl_ros/transforms.h>  // to check transforms
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>  // for cv to eigen ish
#include <opencv2/opencv.hpp>      // for cv::Mat

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
    , mock_localization_pub(handle.advertise<msgs::OdometryConstPtr>("/SOME TOPIC"))
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
    // init node with specific ros params??
    while (!IsNodeReady())
    {
      ros::spinOnce();
    }
  }

  virtual void TearDown()
  {
    // kill node?
  }

  bool IsNodeReady()
  {
    return (mock_map_pub.getNumSubscribers() > 0) && (lidar_sub.getNumPublishers() > 0) &&
           (cam_sub.getNumPublishers() > 0) && (pothole_sub.getNumPublishers() > 0);
  }

  ros::NodeHandle handle;
  ros::Publisher mock_joy_pub;
  ros::Subscriber motor_sub;
};

TEST_Lidar(TestNewMapper, FullForward)
{
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidar_msg;
  pcl::PointXYZ point(1, 1, 1);
  lidar_msg.append(point);
  // init some message to set location
  // don;t know the type some tf trnform message
  // publish some message to change location
  // source code:
  // http://docs.ros.org/diamondback/api/tf/html/c++/tf_8cpp_source.html
  // should affect tf transform
  // nav_msgs::OdometryConstPtr
  mock_lidar_pub.publish(lidar_msg);

  const igvc_msgs::map::ConstPtr &response = ros::topic::waitForMessage<const pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(
      lidar_sub.getTopic(), ros::Duration(1));
  // get point at expected x and y, check for 255 probability
  // makes a sensor_msgs::Image var, need x and y
  // need to go from sensor_msgs::Image to cv::Mat
  int step = response->image.step;
  int expected x = EXPECT_TRUE(response.get() != nullptr);
  EXPECT_EQ(response->image, 1.0);
  EXPECT_EQ(response->right_velocity, 1.0);
}
