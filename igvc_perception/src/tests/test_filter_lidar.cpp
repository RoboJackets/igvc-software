#include <gtest/gtest.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

class TestFilterLidar : public testing::Test
{
  public:
  TestNewMapper()
    : handle()
    , mock_scan_pub(handle.advertise<nav_msgs::Odometry>("/pc2", 1))
    , pointcloud_sub(handle.subscribe("/scan/pointcloud", 1, &TestFilterLidar::pointcloud_callback, this))
  {
  }

  void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>& msg)
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
    return (mock_scan_pub.getNumSubscribers() > 0 && pointcloud_sub.getNumSubscribers() > 0);
  }

  ros::NodeHandle handle;
  ros::Publisher mock_scan_pub;
  ros::Subscriber pointcloud_sub;
};

//TESTs go here
TEST_F(TestPathFollower, Testpxny)
{
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "pc2";


  mock_path_pub.publish(path_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  mock_pose_pub.publish(odom_msg);

  const geometry_msgs::PointStamped::ConstPtr& target_resp =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(target_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(target_resp.get() != nullptr);
  // should be

  const igvc_msgs::velocity_pair::ConstPtr& motors_resp =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motors_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(motors_resp.get() != nullptr);
  // should be
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_filter_lidar");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
