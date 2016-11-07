#include "linedetector.h"
#include <pcl_ros/point_cloud.h>
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

constexpr double radians(double degrees) {
    return degrees / 180.0 * M_PI;
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam.fromCameraInfo(cam_info);
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    src_img = cv_ptr->image;

    cv::resize(src_img, src_img, cv::Size(524, 524), 0, 0, CV_INTER_AREA);
    fin_img = cv::Mat::zeros(src_img.size(), src_img.type());

    cv::GaussianBlur(src_img, working, cv::Size(0,0), 2.0);

    cv::Canny(working, working, 45, 135, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(working, lines, 1.0, CV_PI/180, 80, 35, 15);
    for (size_t i = 0; i < lines.size(); ++i) {
        line(fin_img, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 255), 3, 8);
    }

    cloud = toPointCloud(fin_img);
    _line_cloud.publish(cloud);

    cv_ptr->image = fin_img;
    _filt_img.publish(cv_ptr->toImageMsg());
}

LineDetector::LineDetector(ros::NodeHandle &handle, const std::string& topic)
      : _it(handle)
      , tf_listener(handle)
      , topic(topic)
{
     std::cout<<"Running"<<std::endl;
    _src_img = _it.subscribeCamera(topic + "/image_raw", 1, &LineDetector::img_callback, this);
    _filt_img = _it.advertise(topic + "/filt_img", 1);
    _line_cloud = handle.advertise<PCLCloud>(topic + "/line_cloud", 100);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LineDetector::toPointCloud(cv::Mat img){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", topic, ros::Time(0), transform);
    for(int r = img.rows/2; r < img.rows; r++) {
        for(int c = 0; c < img.cols; c++) {
            if(img.at<uchar>(r, c) > 0) {
                cloud->points.push_back(PointFromPixel(cv::Point(c, r), transform, cam));
            }
        }
    }
    cloud->header.frame_id = "base_footprint";
    return cloud;
}
