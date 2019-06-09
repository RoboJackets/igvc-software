


#include <opencv2/core/mat.hpp>
#include <cv.hpp>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "back_circle");

  using namespace cv;

  //size = 0.2 * i
  double meters = 10;
  int img_size = static_cast<int>(meters / 0.2);
  Mat img(img_size, img_size, CV_8U, Scalar::all(0));
  ellipse(img, Point(img_size/2, img_size/2), Size(5/ 0.2, 5/0.2), 0, 90, 270, Scalar(255), 1 / 0.2);
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/back_circle", 1);

  pcl::PointCloud<pcl::PointXYZ> back_circle;
  back_circle.header.stamp = ros::Time::now().toSec();
  back_circle.header.frame_id = "/base_footprint";


  for(int i=0; i<img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      // You can now access the pixel value with cv::Vec3b
      if(img.at<uchar>(i, j) == 255) {
        back_circle.push_back(pcl::PointXYZ((j - img_size/2) * 0.2, (i - img_size/2) * 0.2, 0));
      }
    }
  }

  // wait to make sure mapper is up
  ros::Duration sleep(3.0);
  sleep.sleep();
  int counter = 0;
  ros::Rate slow(10);
  while(ros::ok() && counter < 10) {
    pub.publish(back_circle);
    counter++;
    slow.sleep();
  }


  return 0;
}
