#include <ros/ros.h>
#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class VisionTut
{
public:
	VisionTut(ros::NodeHandle &handle);

private:
	cv::Mat src_img, fin_img;

	void img_callback(const sensor_msgs::ImageConstPtr& msg);

};