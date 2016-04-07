#include "potholedetector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "potholedetector");

    ros::NodeHandle nh;

	PotholeDetector det{nh};

    ros::spin();

    return 0;
}
