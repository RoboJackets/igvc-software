#include "back_circle_server.h"

BackCircleServer::BackCircleServer()
{
  back_circle_service_server_ =
      nh_.advertiseService("/back_circle_service", &BackCircleServer::backCircleCallback, this);
  ROS_INFO("Back Circle Service Ready");
}

void BackCircleServer::waitForTransform()
{
  const double waiting_time = 5.0;
  while (!tf_listener_.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(waiting_time)))
  {
    ROS_INFO_STREAM("base_footprint->odom transform not found. waiting...");
  }
  ROS_INFO_STREAM("base_footprint->odom transform found!");
}

bool BackCircleServer::backCircleCallback(igvc_msgs::BackCircle::Request &req, igvc_msgs::BackCircle::Response &res)
{
  ros::NodeHandle pNh("~");

  double width, length, grid_size, thickness, offset;
  std::vector<double> xVec, yVec;

  assertions::getParam(pNh, std::string("back_circle/width"), width);
  assertions::getParam(pNh, std::string("back_circle/length"), length);
  assertions::getParam(pNh, std::string("back_circle/grid_size"), grid_size);
  assertions::getParam(pNh, std::string("back_circle/thickness"), thickness);
  assertions::getParam(pNh, std::string("back_circle/offset"), offset);
  double meters = (length + width) * 2;

  int img_size = static_cast<int>(std::round(meters / grid_size));
  cv::Mat img(img_size, img_size, CV_8U, cv::Scalar::all(0));
  cv::ellipse(img, cv::Point(img_size / 2, img_size / 2), cv::Size(width / grid_size, length / grid_size), 0, 90, 270,
              cv::Scalar(255), static_cast<int>(std::round(thickness / grid_size)));

  pcl::PointCloud<pcl::PointXYZ> back_circle_pointcloud_base_footprint;
  back_circle_pointcloud_base_footprint.header.stamp = ros::Time::now().toNSec() / 1000;
  back_circle_pointcloud_base_footprint.header.frame_id = "/base_footprint";

  for (int i = 0; i < img.rows; i++)
  {
    for (int j = 0; j < img.cols; j++)
    {
      if (img.at<uchar>(i, j) == 255)
      {
        pcl::PointXYZ bc_point((j - img_size / 2) * grid_size + offset, (i - img_size / 2) * grid_size, 0);
        back_circle_pointcloud_base_footprint.push_back(bc_point);
      }
    }
  }

  waitForTransform();
  pcl::PointCloud<pcl::PointXYZ> back_circle_pointcloud_odom;
  pcl_ros::transformPointCloud("odom", ros::Time::now(), back_circle_pointcloud_base_footprint, "odom",
                               back_circle_pointcloud_odom, tf_listener_);

  for (size_t i = 0; i < back_circle_pointcloud_odom.points.size(); ++i)
  {
    xVec.push_back(back_circle_pointcloud_odom.points[i].x);
    yVec.push_back(back_circle_pointcloud_odom.points[i].y);
  }

  res.x = xVec;
  res.y = yVec;

  ROS_INFO("Back circle points created.");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "back_circle");
  BackCircleServer node = BackCircleServer();
  ros::spin();
  return 0;
}