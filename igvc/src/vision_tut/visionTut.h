#include <ros/ros.h>
#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

class VisionTut
{
public:
	VisionTut(ros::NodeHandle &handle);

private:
	cv::Mat src_img, working, fin_img;

	void img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);

	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	cv_bridge::CvImagePtr cv_ptr;
    image_geometry::PinholeCameraModel cam;
    image_transport::CameraSubscriber _src_img;
};