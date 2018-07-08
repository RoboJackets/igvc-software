#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class TestPathFollower : public testing::Test
{
public:
  TestPathFollower()
    : handle()
    , mock_path_pub(handle.advertise<nav_msgs::Path>("/path", 1))
    , mock_pose_pub(handle.advertise<nav_msgs::Odometry>("/odometry/filtered", 1))
    , target_sub(handle.subscribe("/target_point", 1, &TestPathFollower::targetCallback, this))
    , motors_sub(handle.subscribe("/motors", 1, &TestPathFollower::motorsCallback, this))
  //, trajectory_sub(handle.subscribe("/trajectory", 1, &TestPathFollower::trajectoryCallback, this))
  {
  }

  /*void trajectoryCallback(const nav_msgs::PathConstPtr& msg)
  {
  }*/

  void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
  }

  void motorsCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
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
    return (mock_path_pub.getNumSubscribers() > 0) && (mock_pose_pub.getNumSubscribers() > 0) &&
           (target_sub.getNumPublishers() > 0) && (motors_sub.getNumPublishers() > 0);
    /*&& (trajectory_sub.getNumPublishers() > 0);*/
  }

  ros::NodeHandle handle;
  ros::Publisher mock_path_pub;
  ros::Publisher mock_pose_pub;
  ros::Subscriber target_sub;
  ros::Subscriber motors_sub;
  // ros::Subscriber trajectory_sub;
};

/*TEST_F(TestPathFollower, Testpx)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "odom";

  geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = path_msg.header.stamp;
  pose1.header.frame_id = path_msg.header.frame_id;
  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  path_msg.poses.push_back(pose1);

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = path_msg.header.stamp;
  pose2.header.frame_id = path_msg.header.frame_id;
  pose2.pose.position.x = 5;
  pose2.pose.position.y = 0;
  path_msg.poses.push_back(pose2);

  mock_path_pub.publish(path_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  mock_pose_pub.publish(odom_msg);

  const geometry_msgs::PointStamped::ConstPtr& target_resp =
ros::topic::waitForMessage<geometry_msgs::PointStamped>(target_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(target_resp.get() != nullptr);
  // should be 2.0

  const igvc_msgs::velocity_pair::ConstPtr& motors_resp =
ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motors_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(motors_resp.get() != nullptr);
  // should be 1.0 1.0
}

TEST_F(TestPathFollower, Testpxpy)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "odom";

  geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = path_msg.header.stamp;
  pose1.header.frame_id = path_msg.header.frame_id;
  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  path_msg.poses.push_back(pose1);

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = path_msg.header.stamp;
  pose2.header.frame_id = path_msg.header.frame_id;
  pose2.pose.position.x = 3.54;
  pose2.pose.position.y = 3.54;
  path_msg.poses.push_back(pose2);

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
  // should be 0.804569 1.19543
}

TEST_F(TestPathFollower, Testpy)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "odom";

  geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = path_msg.header.stamp;
  pose1.header.frame_id = path_msg.header.frame_id;
  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  path_msg.poses.push_back(pose1);

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = path_msg.header.stamp;
  pose2.header.frame_id = path_msg.header.frame_id;
  pose2.pose.position.x = 0;
  pose2.pose.position.y = 5.0;
  path_msg.poses.push_back(pose2);

  mock_path_pub.publish(path_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  mock_pose_pub.publish(odom_msg);

  const geometry_msgs::PointStamped::ConstPtr& target_resp =
ros::topic::waitForMessage<geometry_msgs::PointStamped>(target_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(target_resp.get() != nullptr);
  // should be (0,2)

  const igvc_msgs::velocity_pair::ConstPtr& motors_resp =
ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motors_sub.getTopic(), ros::Duration(1));
  ASSERT_TRUE(motors_resp.get() != nullptr);
  // should be 0.84456 1.155435
}*/

TEST_F(TestPathFollower, Testpxny)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "odom";

  geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = path_msg.header.stamp;
  pose1.header.frame_id = path_msg.header.frame_id;
  pose1.pose.position.x = 0;
  pose1.pose.position.y = 0;
  path_msg.poses.push_back(pose1);

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = path_msg.header.stamp;
  pose2.header.frame_id = path_msg.header.frame_id;
  pose2.pose.position.x = 3.54;
  pose2.pose.position.y = -3.54;
  path_msg.poses.push_back(pose2);

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
  ros::init(argc, argv, "test_path_follower");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
