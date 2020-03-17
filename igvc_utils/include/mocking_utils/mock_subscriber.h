//
// Created by vivek on 3/1/20.
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/**
 * \brief A wrapper class around a Subscriber for testing.
 *
 * A MockMotorSubscriber contains helper methods such as blocking until there is a publisher
 * and spinning until a message arrives.
 *
 * A user provided callback function can be called to perform additional testing logic on a callback
 *
 * Can only use one type of message. Should be able to use any type of message.
 */

template <typename M>
class MockMotorSubscriber
{
public:
//    using MessageType = igvc_msgs::velocity_pair;
//    using CBFunctionType = std::function<void(const MessageType&)>;

    using MessageType = M;
    using CBFunctionType = std::function<void(const MessageType&)>;

    MockMotorSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size = 1);
    MockMotorSubscriber(const std::string& topic, int queue_size = 1);
    ~MockMotorSubscriber();
    MockMotorSubscriber(const MockMotorSubscriber&) = delete;
    MockMotorSubscriber& operator=(const MockMotorSubscriber&) = delete;

    void init(const std::string& topic, int queue_size = 1);

    [[nodiscard]] bool waitForPublisher(const ros::Duration& timeout = ros::Duration{ 5 }) const;
    bool spinUntilMessages(const ros::Duration& timeout = ros::Duration{ 5 }, int num_messages = 1);
    [[nodiscard]] bool hasMessage() const;
    [[nodiscard]] const MessageType& front() const;
    [[nodiscard]] const std::vector<MessageType>& messages() const;

private:
    void callback(const typename MessageType::ConstPtr& message);
    std::optional<CBFunctionType> cb_function_;
    ros::Subscriber sub_;
    ros::CallbackQueue callback_queue_;

    std::vector<MessageType> message_buffer_;
};

sensor_msgs::Joy createJoyMsg(float left, float right);

template <typename M>
MockMotorSubscriber<M>::MockMotorSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size)
        : cb_function_{ std::move(cb_function) }
{
    init(topic, queue_size);
}

template <typename M>
MockMotorSubscriber<M>::MockMotorSubscriber(const std::string& topic, int queue_size)
{
    init(topic, queue_size);
}

template <typename M>
MockMotorSubscriber<M>::~MockMotorSubscriber()
{
    sub_.shutdown();
}

template <typename M>
void MockMotorSubscriber<M>::init(const std::string& topic, int queue_size)
{
    ros::NodeHandle nh;
    ros::SubscribeOptions ops;
    ops.template init<igvc_msgs::velocity_pair>(topic, queue_size, boost::bind(&MockMotorSubscriber::callback, this, _1));
    ops.callback_queue = &callback_queue_;
    sub_ = nh.subscribe(ops);
}

template <typename M>
void MockMotorSubscriber<M>::callback(const typename MessageType::ConstPtr& message)
{
    message_buffer_.emplace_back(*message);
    if (cb_function_)
    {
        cb_function_.value()(*message);
    }
}

template <typename M>
bool MockMotorSubscriber<M>::waitForPublisher(const ros::Duration& timeout) const
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

template <typename M>
bool MockMotorSubscriber<M>::hasMessage() const
{
    return !message_buffer_.empty();
}

template <typename M>
const typename MockMotorSubscriber<M>::MessageType& MockMotorSubscriber<M>::front() const
{
    return message_buffer_.front();
}

template <typename M>
const std::vector<typename MockMotorSubscriber<M>::MessageType>& MockMotorSubscriber<M>::messages() const
{
    return message_buffer_;
}

template <typename M>
bool MockMotorSubscriber<M>::spinUntilMessages(const ros::Duration& timeout, int num_messages)
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