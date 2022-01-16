#include <gtest/gtest.h>
#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mocking_utils/mock_subscriber.h>

class TestSwerveDrive : public testing::Test
{
public:
  TestSwerveDrive() : mock_pub(handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1))
  {
  }

protected:
  ros::NodeHandle handle;
  ros::Publisher mock_pub;
};

geometry_msgs::Twist createTwistMsg(float xVel, float yVel, float spin)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = spin;
  twist_msg.linear.x = xVel;
  twist_msg.linear.y = yVel;
  twist_msg.linear.z = 0.0;

  return twist_msg;
}

TEST_F(TestSwerveDrive, StopTest)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  const float stop = 0.0;
  mock_pub.publish(createTwistMsg(stop, stop, stop));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, stop);
  EXPECT_EQ(response.fr_velocity, stop);
  EXPECT_EQ(response.bl_velocity, stop);
  EXPECT_EQ(response.br_velocity, stop);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveDrive, ForwardTest)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  const float forward = 2.0;
  const float stop = 0.0;
  mock_pub.publish(createTwistMsg(forward, stop, stop));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, forward);
  EXPECT_EQ(response.fr_velocity, forward);
  EXPECT_EQ(response.bl_velocity, forward);
  EXPECT_EQ(response.br_velocity, forward);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}


TEST_F(TestSwerveDrive, TurnTest)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  const float stop = 0.0;
  const float spin = 1.0;

  const geometry_msgs::Twist twist_msg = createTwistMsg(stop, stop, spin);

  mock_pub.publish(twist_msg);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_NEAR(response.fl_velocity, -0.4079, 0.001);
  EXPECT_NEAR(response.fr_velocity, 0.4079, 0.001);
  EXPECT_NEAR(response.bl_velocity, -0.4079, 0.001);
  EXPECT_NEAR(response.br_velocity, 0.4079, 0.001);
  EXPECT_NEAR(response.fl_angle, -1.092, 0.001);
  EXPECT_NEAR(response.fr_angle, 1.092, 0.001);
  EXPECT_NEAR(response.bl_angle, 1.092, 0.001);
  EXPECT_NEAR(response.br_angle, -1.092, 0.001);
}

TEST_F(TestSwerveDrive, MaxSpeedTest)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  const float forward = 5.0;
  const float stop = 0.0;

  double max_vel_;
  handle.getParam("swerve_drive/max_vel", max_vel_);

  const geometry_msgs::Twist twist_msg = createTwistMsg(forward, stop, stop);

  mock_pub.publish(twist_msg);

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, max_vel_);
  EXPECT_EQ(response.fr_velocity, max_vel_);
  EXPECT_EQ(response.bl_velocity, max_vel_);
  EXPECT_EQ(response.br_velocity, max_vel_);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_swerve_drive");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
