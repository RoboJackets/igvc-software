#include <ros/callback_queue.h>
#include <ros/ros.h>
/**
 * \brief A wrapper class around a Subscriber for testing.
 *
 * A MockSubscriber contains helper methods such as blocking until there is a publisher
 * and spinning until a message arrives.
 *
 * A user provided callback function can be called to perform additional testing logic on a callback
 *
 * Able to use any type of message.
 */

template <typename M>
class MockSubscriber
{
public:
  using MessageType = M;
  using CBFunctionType = std::function<void(const MessageType&)>;

  MockSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size = 1);
  MockSubscriber(const std::string& topic, int queue_size = 1);
  ~MockSubscriber();
  MockSubscriber(const MockSubscriber&) = delete;
  MockSubscriber& operator=(const MockSubscriber&) = delete;

  void init(const std::string& topic, int queue_size = 1);

  [[nodiscard]] bool waitForPublisher(const ros::Duration& timeout = ros::Duration{ 5 }) const;
  bool spinUntilMessages(const ros::Duration& timeout = ros::Duration{ 5 }, int num_messages = 1);
  [[nodiscard]] bool hasMessage() const;
  [[nodiscard]] const MessageType& front() const;
  [[nodiscard]] const std::vector<MessageType>& messages() const;
  [[nodiscard]] bool waitForSubscriber(const ros::Publisher& mock_pub) const;

private:
  void callback(const typename MessageType::ConstPtr& message);
  std::optional<CBFunctionType> cb_function_;
  ros::Subscriber sub_;
  ros::CallbackQueue callback_queue_;
  std::vector<MessageType> message_buffer_;
};

template <typename M>
MockSubscriber<M>::MockSubscriber(const std::string& topic, CBFunctionType cb_function, int queue_size)
  : cb_function_{ std::move(cb_function) }
{
  init(topic, queue_size);
}

template <typename M>
MockSubscriber<M>::MockSubscriber(const std::string& topic, int queue_size)
{
  init(topic, queue_size);
}

template <typename M>
MockSubscriber<M>::~MockSubscriber()
{
  sub_.shutdown();
}

template <typename M>
void MockSubscriber<M>::init(const std::string& topic, int queue_size)
{
  ros::NodeHandle nh;
  ros::SubscribeOptions ops;
  ops.template init<M>(topic, queue_size, boost::bind(&MockSubscriber::callback, this, _1));
  ops.callback_queue = &callback_queue_;
  sub_ = nh.subscribe(ops);
}

template <typename M>
void MockSubscriber<M>::callback(const typename MessageType::ConstPtr& message)
{
  message_buffer_.emplace_back(*message);
  if (cb_function_)
  {
    cb_function_.value()(*message);
  }
}

template <typename M>
bool MockSubscriber<M>::waitForPublisher(const ros::Duration& timeout) const
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
[[nodiscard]] bool MockSubscriber<M>::waitForSubscriber(const ros::Publisher& mock_pub) const
{
  const double timeout = 5.0;
  const double sleep_time = 1.0;
  ros::Time end = ros::Time::now() + ros::Duration(timeout);
  while (mock_pub.getNumSubscribers() == 0)
  {
    ros::Duration(sleep_time).sleep();
    if (ros::Time::now() > end)
    {
      return false;
    }
  }
  return true;
}

template <typename M>
bool MockSubscriber<M>::hasMessage() const
{
  return !message_buffer_.empty();
}

template <typename M>
const typename MockSubscriber<M>::MessageType& MockSubscriber<M>::front() const
{
  return message_buffer_.front();
}

template <typename M>
const std::vector<typename MockSubscriber<M>::MessageType>& MockSubscriber<M>::messages() const
{
  return message_buffer_;
}

template <typename M>
bool MockSubscriber<M>::spinUntilMessages(const ros::Duration& timeout, int num_messages)
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