//
// Created by matt on 1/29/16.
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class TestJoystickDriver : public testing::Test
{
public:
  TestJoystickDriver()
    : handle()
    , mock_joy_pub(handle.advertise<sensor_msgs::Joy>("/joy", 1))
    , motor_sub(handle.subscribe("/motors", 1, &TestJoystickDriver::motorsCallback, this))
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
    ros::Duration wait_duration{ 2.0 };
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = { 0, 0.0, 0, 0, 0.0 };
    joy_msg.buttons = { 0, 0, 0, 0 };
    mock_joy_pub.publish(joy_msg);
    wait_duration.sleep();
  }

  virtual void TearDown()
  {
  }

  bool IsNodeReady()
  {
    return (mock_joy_pub.getNumSubscribers() > 0) && (motor_sub.getNumPublishers() > 0);
  }

  ros::NodeHandle handle;
  ros::Publisher mock_joy_pub;
  ros::Subscriber motor_sub;
};

TEST_F(TestJoystickDriver, FullForward)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { 0, 1.0, 0, 0, 0.0 };
  joy_msg.buttons = { 0, 0, 0, 0 };
  mock_joy_pub.publish(joy_msg);

  ros::Duration wait_duration{ 0.1 };
  wait_duration.sleep();

  const igvc_msgs::velocity_pair::ConstPtr response =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motor_sub.getTopic(), ros::Duration(10));

  EXPECT_TRUE(response.get() != nullptr);
  EXPECT_LT(response->left_velocity, 1.0);
  EXPECT_LT(response->right_velocity, 1.0);
  EXPECT_GT(response->left_velocity, 0.0);
  EXPECT_GT(response->right_velocity, 0.0);
  EXPECT_FLOAT_EQ(response->left_velocity, response->right_velocity);
}

TEST_F(TestJoystickDriver, FullReverse)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { 0, -1.0, 0, 0, -1.0 };
  joy_msg.buttons = { 0, 0, 0, 0 };
  mock_joy_pub.publish(joy_msg);

  ros::Duration wait_duration{ 0.1 };
  wait_duration.sleep();

  const igvc_msgs::velocity_pair::ConstPtr response =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motor_sub.getTopic(), ros::Duration(10));

  ASSERT_TRUE(response.get() != nullptr);
  EXPECT_GT(response->left_velocity, -1.0);
  EXPECT_GT(response->right_velocity, -1.0);
  EXPECT_LT(response->left_velocity, 0.0);
  EXPECT_LT(response->right_velocity, 0.0);
  EXPECT_FLOAT_EQ(response->left_velocity, response->right_velocity);
}

TEST_F(TestJoystickDriver, SpinRight)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { -1.0, 0.0, 0, 0, 0.0 };
  joy_msg.buttons = { 0, 0, 0, 0 };
  mock_joy_pub.publish(joy_msg);

  ros::Duration wait_duration{ 0.5 };
  wait_duration.sleep();

  const igvc_msgs::velocity_pair::ConstPtr response =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motor_sub.getTopic(), ros::Duration(10));

  ASSERT_TRUE(response != nullptr && response.get() != nullptr);
  EXPECT_GT(response->left_velocity, 0.0);
  EXPECT_LT(response->right_velocity, 0.0);
}

TEST_F(TestJoystickDriver, SpinLeft)
{
  sensor_msgs::Joy joy_msg;
  joy_msg.axes = { 1.0, 0.0, 0, 0, 0.0 };
  joy_msg.buttons = { 0, 0, 0, 0 };
  mock_joy_pub.publish(joy_msg);

  ros::Duration wait_duration{ 0.5 };
  wait_duration.sleep();

  const igvc_msgs::velocity_pair::ConstPtr response =
      ros::topic::waitForMessage<igvc_msgs::velocity_pair>(motor_sub.getTopic(), ros::Duration(10));

  ASSERT_TRUE(response != nullptr && response.get() != nullptr);
  EXPECT_LT(response->left_velocity, 0.0);
  EXPECT_GT(response->right_velocity, 0.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joystick_driver");
  ros::start();

  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
