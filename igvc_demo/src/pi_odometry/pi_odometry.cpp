#include <pigpio.h>

#include <igvc_msgs/velocity_pair.h>
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

int g_pin_left_a;
int g_pin_left_b;
int g_pin_right_a;
int g_pin_right_b;

int g_left_count_last = 0;
int g_right_count_last = 0;

int g_left_count = 0;
int g_right_count = 0;

double g_counts_to_linear;
ros::Time g_last_time;

ros::Publisher g_encoder_pub;

/**
 * Callback for either edge for left encoder
 * @param gpio
 * @param level
 * @param tick
 */
void left_a_cb(int /*gpio*/, int /*level*/, uint32_t /*tick*/)
{
  auto left_a = gpioRead(g_pin_left_a);
  if (left_a == PI_BAD_GPIO)
  {
    throw std::runtime_error{ "PI_BAD_GPIO from doing gpioRead(g_pin_left_a)" };
  }

  auto left_b = gpioRead(g_pin_left_b);
  if (left_b == PI_BAD_GPIO)
  {
    throw std::runtime_error{ "PI_BAD_GPIO from doing gpioRead(g_pin_left_b)" };
  }

  g_left_count += left_a ^ left_b;
}

/**
 * Callback for either edge for right encoder
 * @param gpio
 * @param level
 * @param tick
 */
void right_a_cb(int /*gpio*/, int /*level*/, uint32_t /*tick*/)
{
  auto right_a = gpioRead(g_pin_right_a);
  if (right_a == PI_BAD_GPIO)
  {
    throw std::runtime_error{ "PI_BAD_GPIO from doing gpioRead(g_pin_right_a)" };
  }

  auto right_b = gpioRead(g_pin_right_b);
  if (right_b == PI_BAD_GPIO)
  {
    throw std::runtime_error{ "PI_BAD_GPIO from doing gpioRead(g_pin_right_b)" };
  }

  g_right_count += right_a ^ right_b;
}

void setupGPIOPins()
{
  if (gpioInitialise() < 0)
  {
    throw std::runtime_error{ "GPIO failed to initialise!" };
  }

  gpioSetMode(g_pin_left_a, PI_INPUT);
  gpioSetMode(g_pin_left_b, PI_INPUT);
  gpioSetMode(g_pin_right_a, PI_INPUT);
  gpioSetMode(g_pin_right_b, PI_INPUT);

  gpioSetISRFunc(g_pin_left_a, EITHER_EDGE, 0, left_a_cb);
  gpioSetISRFunc(g_pin_right_b, EITHER_EDGE, 0, right_a_cb);
}

void publishEncoders()
{
  igvc_msgs::velocity_pair velocity_pair;

  int d_counts_left = g_left_count - g_left_count_last;
  int d_counts_right = g_right_count - g_right_count_last;

  double left_distance = d_counts_left * g_counts_to_linear;
  double right_distance = d_counts_right * g_counts_to_linear;

  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - g_last_time;

  double left_velocity = left_distance / delta_time.toSec();
  double right_velocity = right_distance / delta_time.toSec();

  velocity_pair.left_velocity = left_velocity;
  velocity_pair.right_velocity = right_velocity;
  velocity_pair.header.stamp = current_time;
  velocity_pair.duration = delta_time.toSec();

  g_encoder_pub.publish(velocity_pair);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pi_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{ "~" };

  assertions::getParam(private_nh, "pin/left/a", g_pin_left_a);
  assertions::getParam(private_nh, "pin/left/b", g_pin_left_b);
  assertions::getParam(private_nh, "pin/right/a", g_pin_right_a);
  assertions::getParam(private_nh, "pin/right/b", g_pin_right_b);

  assertions::getParam(private_nh, "pin/right/b", g_pin_right_b);

  double rate;
  assertions::getParam(private_nh, "publish_rate", rate);

  std::string topics_encoders;
  assertions::getParam(private_nh, "topics/encoders", topics_encoders);

  double gear_ratio;
  int cpr;
  double diameter;
  assertions::getParam(private_nh, "gear_ratio", gear_ratio);
  assertions::getParam(private_nh, "diameter", diameter);
  assertions::getParam(private_nh, "cpr", cpr);

  double ppr = cpr / 4.0;
  double circumference = M_PI * diameter;
  g_counts_to_linear = circumference / (gear_ratio * ppr);

  g_encoder_pub = nh.advertise<igvc_msgs::velocity_pair>(topics_encoders, 1);

  setupGPIOPins();

  g_last_time = ros::Time::now();

  ros::Rate r{ rate };
  while (ros::ok())
  {
    publishEncoders();
    r.sleep();
    ros::spinOnce();
  }

  gpioTerminate();
  return 0;
}
