#include "linedetector.h"
#include <pcl_ros/point_cloud.h>
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void LineDetector::info_img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam.fromCameraInfo(cam_info);
    img_callback(msg);
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
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
     if (!hasInfo) {
        _src_img = _it.subscribe("stereo/left/image_raw", 1, &LineDetector::img_callback, this);
     } else {
        _src_img_info = _it.subscribeCamera(topic + "/image_raw", 1, &LineDetector::info_img_callback, this);
     }
    _filt_img = _it.advertise(topic + "/filt_img", 1);
    _line_cloud = handle.advertise<PCLCloud>(topic + "/line_cloud", 100);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LineDetector::toPointCloud(cv::Mat img){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera_left", ros::Time(0), transform);
    for(int r = img.rows/2; r < img.rows; r++) {
        for(int c = 0; c < img.cols; c++) {
            if(img.at<uchar>(r, c) > 0) {
                if (!hasInfo) {
                    double roll, pitch, yaw;
                    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
                    double origin_z = transform.getOrigin().getZ();
                    double origin_y = transform.getOrigin().getY();
                    double HFOV = toRadians(66.0);
                    double VFOV = toRadians(47.6);
                    pitch = -roll;
                    cloud->points.push_back(PointFromPixelNoCam(cv::Point(c, r), img.cols, img.rows, HFOV, VFOV, origin_z, origin_y, pitch));
                } else {
                    cloud->points.push_back(PointFromPixel(cv::Point(c, r), transform, cam));
                }
            }
        }
    }
    cloud->header.frame_id = "base_footprint";
    return cloud;
}
