#include <cv_bridge/cv_bridge.h>  // for eigen to cv ish
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // to check transforms
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/core/eigen.hpp>  // for cv to eigen ish
#include <opencv2/opencv.hpp>      // for cv::Mat

class TestMapper : public testing::Test
{
public:
  TestMapper()
    : handle()
    , mock_localization_pub(handle.advertise<nav_msgs::Odometry>("/odometry/filtered", 1))
    , mock_lidar_pub(handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scan/pointcloud", 1))
    , mock_camera_pub(handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/usb_cam_center/line_cloud", 1))
    , map_sub(handle.subscribe("/map", 1, &TestMapper::map_callback, this))
  {
  }

  void map_callback(const igvc_msgs::mapConstPtr& msg)
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
    return (mock_camera_pub.getNumSubscribers() > 0) && (mock_localization_pub.getNumSubscribers() > 0) &&
           (mock_lidar_pub.getNumSubscribers() > 0) && (map_sub.getNumPublishers() > 0);
  }

  ros::NodeHandle handle;
  ros::Publisher mock_localization_pub;
  ros::Publisher mock_lidar_pub;
  ros::Publisher mock_camera_pub;
  ros::Subscriber map_sub;
};

TEST_F(TestMapper, OriginCheck)
{
  tf::TransformBroadcaster br;
  tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  transform.child_frame_id_ = "/base_footprint";
  transform.frame_id_ = "/lidar";
  transform.stamp_ = ros::Time::now();
  br.sendTransform(transform);

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  mock_localization_pub.publish(odom_msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_msg;
  pcl::PointXYZ point(0, 1, 0);
  cloud_msg.points.push_back(point);
  cloud_msg.header.frame_id = "/lidar";
  mock_lidar_pub.publish(cloud_msg);

  // http://docs.ros.org/diamondback/api/tf/html/c++/tf_8cpp_source.html

  const igvc_msgs::map::ConstPtr& response =
      ros::topic::waitForMessage<igvc_msgs::map>(map_sub.getTopic(), ros::Duration(5));
  // get point at expected x and y, check for 255 probability
  // makes a sensor_msgs::Image var, need x and y
  // need to go from sensor_msgs::Image to cv::Mat
  ASSERT_TRUE(response.get() != nullptr);
  const sensor_msgs::Image img = response->image;
  cv::Mat map = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
  ASSERT_EQ(map.at<uchar>(250, 255), (uchar)125);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_mapper");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
