#include "visionTut.h"

VisionTut::VisionTut(ros::NodeHandle &handle)
	: it(handle)
{
	_src_img = it.subscribeCamera("/stereo/left/image_raw", 1, &VisionTut::img_callback, this);
	pub = it.advertise("/stereo/left/filt_img", 1);
}

void VisionTut::img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    cam.fromCameraInfo(cam_info);
	cv_ptr = cv_bridge::toCvCopy(msg, "");
	// src_img = cv_ptr->image;
	// cv::GaussianBlur(src_img, fin_img, cv::Size(0,0), 2.0);
	// cv_ptr->image = fin_img;
	pub.publish(cv_ptr->toImageMsg());
}