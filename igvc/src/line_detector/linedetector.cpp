#include "linedetector.h"
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <queue>
#include <chrono>
#include <ctime>

using namespace std;
using namespace cv;
using namespace pcl;


cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

constexpr double radians(double degrees) {
    return degrees / 180.0 * M_PI;
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cam.fromCameraInfo(cam_info);
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    src_img = cv_ptr->image;

    resize(src_img, src_img, Size(524, 524), 0, 0, CV_INTER_AREA);
    fin_img = Mat::zeros(src_img.size(), src_img.type());

    GaussianBlur(src_img, working, Size(0,0), 2.0);

    Canny(working, working, 45, 135, 3);

    vector<Vec4i> lines;
    HoughLinesP(working, lines, 1.0, CV_PI/180, 80, 35, 15);
    for (size_t i = 0; i < lines.size(); ++i) {
        line(fin_img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 255, 255), 3, 8);
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
     cout<<"Running"<<endl;
    _src_img = _it.subscribeCamera(topic + "/image_raw", 1, &LineDetector::img_callback, this);
    _filt_img = _it.advertise(topic + "/filt_img", 1);
    _line_cloud = handle.advertise<PCLCloud>(topic + "/line_cloud", 100);
}

PointCloud<PointXYZ>::Ptr LineDetector::toPointCloud(Mat img){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", topic, ros::Time(0), transform);
    for(int r = img.rows/2; r < img.rows; r++) {
        for(int c = 0; c < img.cols; c++) {
            if(img.at<uchar>(r, c) > 0) {
                cloud->points.push_back(PointFromPixel(Point(c, r), transform));
            }
        }
    }
    cloud->header.frame_id = "base_footprint";
    return cloud;
}

// @todo add this to a util class
pcl::PointXYZ LineDetector::PointFromPixel(const cv::Point& pixel, const tf::Transform& cameraFrameToWorldFrame) {
    cv::Point3d cameraRay = cam.projectPixelTo3dRay(pixel);
    tf::Point worldCameraOrigin = cameraFrameToWorldFrame * tf::Vector3(0, 0, 0);
    tf::Point worldCameraStep = cameraFrameToWorldFrame * tf::Vector3(cameraRay.x, cameraRay.y, cameraRay.z) - worldCameraOrigin;
    double zScale = -worldCameraOrigin.z()/worldCameraStep.z();
    tf::Point ret = worldCameraOrigin + zScale * worldCameraStep;
    return pcl::PointXYZ(ret.x(), ret.y(), 0);
}
