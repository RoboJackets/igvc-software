#include <parameter_assertions/assertions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <cv.hpp>
#include <opencv2/core/mat.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "back_circle");

  ros::NodeHandle pNh("~");

  double width, length, grid_size, thickness, offset;
  // parameters

  assertions::getParam(pNh, std::string("width"), width);
  assertions::getParam(pNh, std::string("length"), length);
  assertions::getParam(pNh, std::string("grid_size"), grid_size);
  assertions::getParam(pNh, std::string("thickness"), thickness);
  assertions::getParam(pNh, std::string("offset"), offset);

  using namespace cv;

  // size = 0.2 * i
  double meters = (length + width) * 2;
  int img_size = static_cast<int>(std::round(meters / grid_size));
  Mat img(img_size, img_size, CV_8U, Scalar::all(0));
  ellipse(img, Point(img_size / 2, img_size / 2), Size(width / grid_size, length / grid_size), 0, 90, 270, Scalar(255),
          static_cast<int>(std::round(thickness / grid_size)));
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/back_circle", 1);

  pcl::PointCloud<pcl::PointXYZ> back_circle;
  back_circle.header.stamp = ros::Time::now().toSec();
  back_circle.header.frame_id = "/base_footprint";

  for (int i = 0; i < img.rows; i++)
  {
    for (int j = 0; j < img.cols; j++)
    {
      // You can now access the pixel value with cv::Vec3b
      if (img.at<uchar>(i, j) == 255)
      {
        back_circle.push_back(
            pcl::PointXYZ((j - img_size / 2) * grid_size + offset, (i - img_size / 2) * grid_size, 0));
      }
    }
  }

  // wait to make sure mapper is up
  ros::Duration sleep(3.0);
  sleep.sleep();
  int counter = 0;
  ros::Rate slow(10);
  while (ros::ok() && counter < 10)
  {
    pub.publish(back_circle);
    counter++;
    slow.sleep();
  }

  return 0;
}
