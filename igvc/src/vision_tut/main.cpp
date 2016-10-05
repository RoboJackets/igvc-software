#include "visionTut.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "visionTut");

	ros::NodeHandle nh;

	VisionTut vis(nh);

	ros::spin();

	return 0;
}