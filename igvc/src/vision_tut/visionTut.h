#include <ros/ros.h>
#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class VisionTut
{
public:
	VisionTut(ros::NodeHandle &handle);

private:
	cv::Mat src_img, working, fin_img;

	void img_callback(const sensor_msgs::ImageConstPtr& msg);

	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	cv_bridge::CvImagePtr cv_ptr;
    image_transport::Subscriber sub;
};