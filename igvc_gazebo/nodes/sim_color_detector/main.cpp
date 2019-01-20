#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher img_pub;


void handle_image(const sensor_msgs::ImageConstPtr& msg) {
  // convert to cv matrix

  /*
    cv_bridge::CvImagePtr cv_ptr;
    Mat frame;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV-Bridge error: %s", e.what());
        return;
    }
    frame = cv_ptr->image;
    Mat output(frame.rows, frame.cols, CV_8UC1, Scalar::all(0));


  */

  // iterate through frame and in ouput
  // color detect for purple purple = white, else black
  // convert back to image message

  /*  sensor_msgs::Image outmsg;
    cv_ptr->image = output;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);
    img_pub.publish(outmsg);
  */

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_color_detector");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string topic_name;
  pNh.param("image_topic", topic_name, std::string("/usb_cam_center/image_raw"));

  ros::Subscriber img_sub = nh.subscribe(topic_name, 1, handle_image);

  img_pub = nh.advertise<sensor_msgs::Image>("/usb_cam_center/detected", 1);

  ros::spin();

  return 0;

}
