#include "YostLabDriver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  YostLabDriver imu_drv(nh, priv_nh);
  imu_drv.run();
}
