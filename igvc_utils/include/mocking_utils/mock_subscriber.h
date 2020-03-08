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

    [[nodiscard]] bool waitForPublisher(const ros::Duration& timeout = ros::Duration{ 5 }) const;
    bool spinUntilMessages(const ros::Duration& timeout = ros::Duration{ 5 }, int num_messages = 1);
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

sensor_msgs::Joy createJoyMsg(float left, float right);