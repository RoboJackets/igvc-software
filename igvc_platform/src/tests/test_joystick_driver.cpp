//
// Created by matt on 1/29/16.
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

// TODO: Make this templated

/**
 * \brief A wrapper class around a Subscriber for testing.
 *
 * A MockMotorSubscriber contains helper methods such as blocking until there is a publisher
 * and spinning until a message arrives.
 *
 * A user provided callback function can be called to perform additional testing logic on a callback
 */
class MockMotorSubscriber
{
public:
  using MessageType = igvc_msgs::velocity_pair;
  using CBFunctionType = std::function<void(const MessageType&)>;

  MockMotorSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size = 1);
  MockMotorSubscriber(const std::string& topic, int queue_size = 1);
  ~MockMotorSubscriber();
  MockMotorSubscriber(const MockMotorSubscriber&) = delete;
  MockMotorSubscriber& operator=(const MockMotorSubscriber&) = delete;

  void init(const std::string& topic, int queue_size = 1);

  [[nodiscard]] bool waitForPublisher(const ros::Duration& timeout = ros::Duration{ 0 }) const;
  bool spinUntilMessages(const ros::Duration& timeout = ros::Duration{ 0 }, int num_messages = 1);
  [[nodiscard]] bool hasMessage() const;
  [[nodiscard]] const MessageType& front() const;
  [[nodiscard]] const std::vector<MessageType>& messages() const;

private:
  void callback(const MessageType::ConstPtr& message);
  std::optional<CBFunctionType> cb_function_;
  ros::Subscriber sub_;
  ros::CallbackQueue callback_queue_;

  std::vector<MessageType> message_buffer_;
};

MockMotorSubscriber::MockMotorSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size)
  : cb_function_{ std::move(cb_function) }
{
  init(topic, queue_size);
}

MockMotorSubscriber::MockMotorSubscriber(const std::string& topic, int queue_size)
{
  init(topic, queue_size);
}

MockMotorSubscriber::~MockMotorSubscriber()
{
  sub_.shutdown();
}

void MockMotorSubscriber::init(const std::string& topic, int queue_size)
{
  ros::NodeHandle nh;
  ros::SubscribeOptions ops;
  ops.template init<igvc_msgs::velocity_pair>(topic, queue_size, boost::bind(&MockMotorSubscriber::callback, this, _1));
  ops.callback_queue = &callback_queue_;
  sub_ = nh.subscribe(ops);
}

void MockMotorSubscriber::callback(const MessageType::ConstPtr& message)
{
  message_buffer_.emplace_back(*message);
  if (cb_function_)
  {
    cb_function_.value()(*message);
  }
}

bool MockMotorSubscriber::waitForPublisher(const ros::Duration& timeout) const
{
  ros::Time end = ros::Time::now() + timeout;
  while (sub_.getNumPublishers() == 0)
  {
    ros::WallDuration(0.1).sleep();
    if (!timeout.isZero() && ros::Time::now() >= end)
    {
      return false;
    }
  }
  return true;
}

bool MockMotorSubscriber::hasMessage() const
{
  return !message_buffer_.empty();
}

const MockMotorSubscriber::MessageType& MockMotorSubscriber::front() const
{
  return message_buffer_.front();
}

const std::vector<MockMotorSubscriber::MessageType>& MockMotorSubscriber::messages() const
{
  return message_buffer_;
}

bool MockMotorSubscriber::spinUntilMessages(const ros::Duration& timeout, int num_messages)
{
  if (sub_.getNumPublishers() == 0)
  {
    return false;
  }

  ros::Time end = ros::Time::now() + timeout;
  while (static_cast<int>(message_buffer_.size()) < num_messages)
  {
    callback_queue_.callAvailable(ros::WallDuration{ 0.1 });
    if (!timeout.isZero() && ros::Time::now() >= end)
    {
      return false;
    }
  }
  return true;
}

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
}

TEST_F(TestJoystickDriver, FullForward)
{
  MockMotorSubscriber mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
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
  MockMotorSubscriber mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
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
  MockMotorSubscriber mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
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
  MockMotorSubscriber mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
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
  MockMotorSubscriber mock_sub("/motors");
  ASSERT_TRUE(mock_sub.waitForPublisher());
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
