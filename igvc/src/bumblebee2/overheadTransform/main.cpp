#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/publisher.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"

using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
image_transport::Publisher _new_img;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat img_normal = (cv_ptr->image).clone();

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    obj.push_back(Point2f(200, 200));
    obj.push_back(Point2f(600, 200));
    obj.push_back(Point2f(200, 600));
    obj.push_back(Point2f(600, 600));

    scene.push_back(Point2f(240, 415));
    scene.push_back(Point2f(725, 415));
    scene.push_back(Point2f(115, 540));
    scene.push_back(Point2f(805, 540));

    // Find Homography
    Mat H = findHomography(scene, obj, CV_RANSAC, 3);

    // Transform Image
    Mat img_transformed;
    warpPerspective(img_normal, img_transformed, H, img_transformed.size());

    cv_ptr->image = img_transformed;
    _new_img.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "overheadTransform");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/left/image_rect_color", 1, callback);

    image_transport::ImageTransport _it(nh);
    _new_img = _it.advertise("/overhead", 1);

    ros::spin();
}
