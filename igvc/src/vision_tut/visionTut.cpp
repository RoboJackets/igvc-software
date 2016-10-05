#include "visionTut.h"

VisionTut::VisionTut(ros::NodeHandle &handle)
	: it(handle)
{
	sub = it.subscribe("/stereo/left/image_raw", 1, &VisionTut::img_callback, this);
	pub = it.advertise("/stereo/left/filt_img", 1);
}

void VisionTut::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_ptr = cv_bridge::toCvCopy(msg, "");
	src_img = cv_ptr->image;

	cv::cvtColor(src_img, working, CV_BGR2GRAY);

	cv::resize(working, working, cv::Size(524, 524), 0, 0, CV_INTER_AREA);
    fin_img = cv::Mat::zeros(src_img.size(), src_img.type());
	
	cv::GaussianBlur(working, working, cv::Size(0,0), 2.0);
	
	cv::Canny(working, working, 45, 45*3, 3);

	std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(working, lines, 1.0, CV_PI/180, 80, 35, 15);
    for (size_t i = 0; i < lines.size(); ++i) {
        cv::line(working, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 255), 3, 8);
    }
	cv::resize(working, working, src_img.size(), 0, 0, CV_INTER_AREA);
    src_img.copyTo(fin_img, working);
	cv_ptr->image = fin_img;
	pub.publish(cv_ptr->toImageMsg());
}