#include <igvc_msgs/system_stats.h>
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <sys/sysinfo.h>
#include "cpu_usage.hpp"

ros::Publisher stats_pub;
struct sysinfo sys_info;
constexpr double conversion_factor = 1 << 30;

/**
 * Converts bytes to gigabytes
 * @param bytes
 * @return gigabytes
 */
inline double to_gigabytes(unsigned long bytes)
{
  return static_cast<double>(bytes / conversion_factor);
}

/**
 * Updates sys_info global variable
 */
void update_sysinfo()
{
  sysinfo(&sys_info);
}

/**
 * @return amount of used memory in GB.
 */
double get_used_memory()
{
  return to_gigabytes(sys_info.totalram - sys_info.freeram);
}

/**
 * @return free memory in GB.
 */
double get_free_memory()
{
  return to_gigabytes(sys_info.freeram);
}

/**
 * @return total memory in GB
 */
double get_total_memory()
{
  return to_gigabytes(sys_info.totalram);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "system_stats");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  stats_pub = nh.advertise<igvc_msgs::system_stats>("system_stats", 10);

  double frequency;

  assertions::param(pNh, "publish_frequency", frequency, 10.0);

  ros::Rate rate(frequency);

  while (ros::ok())
  {
    if (stats_pub.getNumSubscribers() > 0)
    {
      update_sysinfo();
      std::vector<double> usages{};
      igvc_msgs::system_stats stats;
      stats.header.stamp = ros::Time::now();
      stats.used_memory = get_used_memory();
      stats.total_memory = get_total_memory();
      double total = get_usage(usages);
      stats.total_cpu_usage = total;
      stats.cpu = usages;
      stats_pub.publish(stats);
    }
    rate.sleep();
  }

  return 0;
}
