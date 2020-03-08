//
// Created by vivek on 3/1/20.
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mocking_utils/mock_subscriber.h>

/**
 * \brief A wrapper class around a Subscriber for testing.
 *
 * A MockMotorSubscriber contains helper methods such as blocking until there is a publisher
 * and spinning until a message arrives.
 *
 * A user provided callback function can be called to perform additional testing logic on a callback
 */

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
        const double sleep_duration = 0.1;
        ros::WallDuration{ sleep_duration }.sleep();
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
        const double sleep_duration = 0.1;
        callback_queue_.callAvailable(ros::WallDuration{ sleep_duration });
        if (!timeout.isZero() && ros::Time::now() >= end)
        {
            return false;
        }
    }
    return true;
}

sensor_msgs::Joy createJoyMsg(float left, float right)
{
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = { 0, left, 0, 0, right };
    joy_msg.buttons = { 0, 0, 0, 0 };

    return joy_msg;
}