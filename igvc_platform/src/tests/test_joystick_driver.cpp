//
// Created by matt on 1/29/16.
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mocking_utils/mock_subscriber.h>

class TestJoystickDriver : public testing::Test
{
public:
  TestJoystickDriver() : mock_joy_pub(handle.advertise<sensor_msgs::Joy>("/joy", 1))
  {
  }

protected:
  ros::NodeHandle handle;
  ros::Publisher mock_joy_pub;
};

sensor_msgs::Joy createJoyMsg(float left, float right)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { 0, left, 0, 0, right };
  joy_msg.buttons = { 0, 0, 0, 0 };

  return joy_msg;
};

TEST_F(TestJoystickDriver, FullForward)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));

  const float full_speed = 1.0;
  mock_joy_pub.publish(createJoyMsg(full_speed, full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, full_speed);
  EXPECT_EQ(response.right_velocity, full_speed);
}

TEST_F(TestJoystickDriver, FullReverse)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  mock_joy_pub.publish(createJoyMsg(-full_speed, -full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, -full_speed);
  EXPECT_EQ(response.right_velocity, -full_speed);
}

TEST_F(TestJoystickDriver, SpinRight)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  mock_joy_pub.publish(createJoyMsg(full_speed, -full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, full_speed);
  EXPECT_EQ(response.right_velocity, -full_speed);
}

TEST_F(TestJoystickDriver, SpinLeft)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float full_speed = 1.0;
  mock_joy_pub.publish(createJoyMsg(-full_speed, full_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, -full_speed);
  EXPECT_EQ(response.right_velocity, full_speed);
}

TEST_F(TestJoystickDriver, HalfSpeedForward)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float half_speed = 0.5;
  mock_joy_pub.publish(createJoyMsg(half_speed, half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, half_speed);
  EXPECT_EQ(response.right_velocity, half_speed);
}

TEST_F(TestJoystickDriver, HalfSpeedReverse)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float half_speed = 0.5;
  mock_joy_pub.publish(createJoyMsg(-half_speed, -half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, -half_speed);
  EXPECT_EQ(response.right_velocity, -half_speed);
}

TEST_F(TestJoystickDriver, HalfSpinRight)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float half_speed = 0.5;
  mock_joy_pub.publish(createJoyMsg(half_speed, -half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, half_speed);
  EXPECT_EQ(response.right_velocity, -half_speed);
}

TEST_F(TestJoystickDriver, HalfSpinLeft)
{
  MockSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_joy_pub));
  const float half_speed = 0.5;
  mock_joy_pub.publish(createJoyMsg(-half_speed, half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, -half_speed);
  EXPECT_EQ(response.right_velocity, half_speed);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joystick_driver");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
