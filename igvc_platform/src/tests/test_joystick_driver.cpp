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
    [[nodiscard]] bool waitForSubscriber() const {
        const double timeout = 5.0;
        const double sleep_time = 1.0;
        ros::Time end = ros::Time::now() + ros::Duration(timeout);
        while (mock_joy_pub.getNumSubscribers() == 0)
        {
            ros::Duration(sleep_time).sleep();
            if (ros::Time::now() > end)
            {
                return false;
            }
        }
        return true;
    }

    ros::NodeHandle handle;
    ros::Publisher mock_joy_pub;
};

sensor_msgs::Joy createJoyMsg(float left, float right)
{
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = { 0, left, 0, 0, right };
    joy_msg.buttons = { 0, 0, 0, 0 };

    return joy_msg;
}

TEST_F(TestJoystickDriver, FullForward)
{
  MockMotorSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(waitForSubscriber());

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
  MockMotorSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(waitForSubscriber());
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
  MockMotorSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(waitForSubscriber());
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
  MockMotorSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(waitForSubscriber());
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
  MockMotorSubscriber<igvc_msgs::velocity_pair> mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(waitForSubscriber());
  const float half_speed = 0.5;
  mock_joy_pub.publish(createJoyMsg(half_speed, half_speed));

  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_msgs::velocity_pair& response = mock_sub.front();

  EXPECT_EQ(response.left_velocity, half_speed);
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
