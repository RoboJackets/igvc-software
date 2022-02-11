#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <mocking_utils/mock_subscriber.h>

class TestDifferentialDrive : public testing::Test
{
public:
  TestDifferentialDrive() : mock_pub(handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1))
  {
  }

protected:
  ros::NodeHandle handle;
  ros::Publisher mock_pub;
};

geometry_msgs::Twist createTwistMsg(float forward, float spin)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = spin;
  twist_msg.linear.x = forward;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;

  return twist_msg;
}

igvc_msgs::velocity_pair twistToVelocity(geometry_msgs::Twist twist, double axle_length_, double max_vel_)
{
  double speed = twist.linear.x;
  double rotation = twist.angular.z;
  double vel_right = speed + (rotation * axle_length_) / 2;
  double vel_left = speed - (rotation * axle_length_) / 2;

  double max_calc_vel = fmax(vel_right, vel_left);
  if (max_calc_vel > max_vel_)
  {
    vel_right *= max_vel_ / max_calc_vel;
    vel_left *= max_vel_ / max_calc_vel;
  }

  igvc_msgs::velocity_pair vel_msg;
  vel_msg.right_velocity = vel_right;
  vel_msg.left_velocity = vel_left;
  return vel_msg;
}

TEST_F(TestDifferentialDrive, StopTest)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  const float stop = 0.0;
  mock_pub.publish(createTwistMsg(stop, stop));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, stop);
  EXPECT_EQ(response.right_velocity, stop);
}

TEST_F(TestDifferentialDrive, ForwardTest)
{

  const float forward = 2.0;
  mock_pub.publish(createTwistMsg(forward, 0.0));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, forward);
  EXPECT_EQ(response.right_velocity, forward);
}

TEST_F(TestDifferentialDrive, TurnTest)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  double axle_length_;
  handle.getParam("differential_drive/axle_length", axle_length_);
  double max_vel_;
  handle.getParam("differential_drive/max_vel", max_vel_);

  const float forward = 1.0;
  const float spin = 1.0;

  const geometry_msgs::Twist twist_msg = createTwistMsg(forward, spin);

  mock_pub.publish(twist_msg);

  const igvc_msgs::velocity_pair vel_msg = twistToVelocity(twist_msg, axle_length_, max_vel_);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, vel_msg.left_velocity);
  EXPECT_EQ(response.right_velocity, vel_msg.right_velocity);
}

TEST_F(TestDifferentialDrive, SpinTest)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  double axle_length_;
  handle.getParam("differential_drive/axle_length", axle_length_);
  double max_vel_;
  handle.getParam("differential_drive/max_vel", max_vel_);

  const float forward = 0.0;
  const float spin = 1.0;

  const geometry_msgs::Twist twist_msg = createTwistMsg(forward, spin);

  mock_pub.publish(twist_msg);

  const igvc_msgs::velocity_pair vel_msg = twistToVelocity(twist_msg, axle_length_, max_vel_);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, vel_msg.left_velocity);
  EXPECT_EQ(response.right_velocity, vel_msg.right_velocity);
}

TEST_F(TestDifferentialDrive, MaxSpeedTest)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  double axle_length_;
  handle.getParam("differential_drive/axle_length", axle_length_);
  double max_vel_;
  handle.getParam("differential_drive/max_vel", max_vel_);

  const float forward = 4.0;
  const float spin = 1.0;

  const geometry_msgs::Twist twist_msg = createTwistMsg(forward, spin);

  mock_pub.publish(twist_msg);

  const igvc_msgs::velocity_pair vel_msg = twistToVelocity(twist_msg, axle_length_, max_vel_);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, vel_msg.left_velocity);
  EXPECT_EQ(response.right_velocity, vel_msg.right_velocity);
}

TEST_F(TestDifferentialDrive, NoiseIgnoreTest)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  double axle_length_;
  handle.getParam("differential_drive/axle_length", axle_length_);
  double max_vel_;
  handle.getParam("differential_drive/max_vel", max_vel_);

  const float forward = 4.0;
  const float spin = 1.0;

  geometry_msgs::Twist twist_msg;
  twist_msg.angular.x = 2.0;
  twist_msg.angular.y = 3.0;
  twist_msg.angular.z = spin;
  twist_msg.linear.x = forward;
  twist_msg.linear.y = 4.0;
  twist_msg.linear.z = 2.0;

  mock_pub.publish(twist_msg);

  const igvc_msgs::velocity_pair vel_msg = twistToVelocity(twist_msg, axle_length_, max_vel_);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, vel_msg.left_velocity);
  EXPECT_EQ(response.right_velocity, vel_msg.right_velocity);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_differential_drive");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
