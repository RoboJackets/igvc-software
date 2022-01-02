#include <gtest/gtest.h>
#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mocking_utils/mock_subscriber.h>

class TestSwerveJoystickDriver : public testing::Test
{
public:
  TestSwerveJoystickDriver() : mock_joy_pub(handle.advertise<sensor_msgs::Joy>("/joy", 1))
  {
  }

protected:
  ros::NodeHandle handle;
  ros::Publisher mock_joy_pub;
};

sensor_msgs::Joy createJoyMsg(float Left_LR, float Left_UD, float Right_LR, float Right_UD)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { Left_LR, Left_UD, 0, Right_LR, Right_UD };
  joy_msg.buttons = { 0, 0, 0, 0 };

  return joy_msg;
};

TEST_F(TestSwerveJoystickDriver, FullForward)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));

  const float full_speed = 1.0;
  const float stop = 0.0;
  mock_joy_pub.publish(createJoyMsg(0, full_speed, 0, full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, full_speed);
  EXPECT_EQ(response.fr_velocity, full_speed);
  EXPECT_EQ(response.bl_velocity, full_speed);
  EXPECT_EQ(response.br_velocity, full_speed);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveJoystickDriver, FullReverse)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  const float stop = 0.0;
  mock_joy_pub.publish(createJoyMsg(0, -full_speed, 0, -full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, -full_speed);
  EXPECT_EQ(response.fr_velocity, -full_speed);
  EXPECT_EQ(response.bl_velocity, -full_speed);
  EXPECT_EQ(response.br_velocity, -full_speed);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveJoystickDriver, SpinRight)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  const float stop = 0.0;
  mock_joy_pub.publish(createJoyMsg(0, full_speed, 0, -full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, full_speed);
  EXPECT_EQ(response.fr_velocity, -full_speed);
  EXPECT_EQ(response.bl_velocity, full_speed);
  EXPECT_EQ(response.br_velocity, -full_speed);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveJoystickDriver, SpinLeft)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  const float stop = 0.0;
  mock_joy_pub.publish(createJoyMsg(0, -full_speed, 0, full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, -full_speed);
  EXPECT_EQ(response.fr_velocity, full_speed);
  EXPECT_EQ(response.bl_velocity, -full_speed);
  EXPECT_EQ(response.br_velocity, full_speed);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveJoystickDriver, HalfSpeedForward)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));

  const float half_speed = 0.5;
  const float stop = 0.0;
  mock_joy_pub.publish(createJoyMsg(0, half_speed, 0, half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_EQ(response.fl_velocity, half_speed);
  EXPECT_EQ(response.fr_velocity, half_speed);
  EXPECT_EQ(response.bl_velocity, half_speed);
  EXPECT_EQ(response.br_velocity, half_speed);
  EXPECT_EQ(response.fl_angle, stop);
  EXPECT_EQ(response.fr_angle, stop);
  EXPECT_EQ(response.bl_angle, stop);
  EXPECT_EQ(response.br_angle, stop);
}

TEST_F(TestSwerveJoystickDriver, PureSpin)
{
  MockSubscriber<igvc_msgs::velocity_quad> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));

  const float half_speed = 0.5;
  const float turnRight = 1.0;
  const float turn90 = 1.57;
  mock_joy_pub.publish(createJoyMsg(turnRight, half_speed, -turnRight, half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_quad& response = mock_sub.front();

  EXPECT_FLOAT_EQ(response.fl_velocity, half_speed);
  EXPECT_FLOAT_EQ(response.fr_velocity, half_speed);
  EXPECT_FLOAT_EQ(response.bl_velocity, half_speed);
  EXPECT_FLOAT_EQ(response.br_velocity, half_speed);
  EXPECT_FLOAT_EQ(response.fl_angle, turn90);
  EXPECT_FLOAT_EQ(response.fr_angle, turn90);
  EXPECT_FLOAT_EQ(response.bl_angle, -turn90);
  EXPECT_FLOAT_EQ(response.br_angle, -turn90);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_swerve_joystick_driver");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
