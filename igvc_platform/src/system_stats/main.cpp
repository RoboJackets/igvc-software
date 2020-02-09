//
// Created by matt on 5/28/16.
//

#include <glibtop.h>
#include <glibtop/cpu.h>
#include <glibtop/mem.h>
#include <igvc_msgs/system_stats.h>
#include <ros/ros.h>
#include <vector>

ros::Publisher stats_pub;

/**
 * Returns the amount of used RAM in GiB.
 */
double get_used_memory()
{
  glibtop_mem mem;
  glibtop_get_mem(&mem);
  return (double)mem.user / ((double)(G_GUINT64_CONSTANT(1) << 30));
}

/**
 * Returns the CPU load of each core as a percentage.
 */
std::vector<double> get_cpu_usage()
{
  glibtop_cpu cpu;

  glibtop_get_cpu(&cpu);

  std::vector<double> percentages;

  for (guint i = 0; i < GLIBTOP_NCPU; i++)
  {
    if (cpu.xcpu_user[i] > 0)
    {
      auto used = cpu.xcpu_user[i] + cpu.xcpu_nice[i] + cpu.xcpu_sys[i];
      percentages.push_back(((double)used / (double)cpu.xcpu_total[i]) * 100.0);
    }
  }

  return percentages;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "system_state");

  ros::NodeHandle handle;

  stats_pub = handle.advertise<igvc_msgs::system_stats>("system_stats", 10);

  double frequency = 10.0;

  if (handle.hasParam("frequency"))
  {
    handle.getParam("frequency", frequency);
  }

  ros::Rate rate(frequency);

  while (!ros::isShuttingDown())
  {
    if (stats_pub.getNumSubscribers() > 0)
    {
      igvc_msgs::system_stats stats;
      stats.header.stamp = ros::Time::now();
      stats.memory = get_used_memory();
      stats.cpu = get_cpu_usage();
      stats_pub.publish(stats);
    }
    rate.sleep();
  }

  return 0;
}