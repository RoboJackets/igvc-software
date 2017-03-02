#ifndef CVUTILS_H
#define CVUTILS_H

pcl::PointXYZ PointFromPixel(const cv::Point& pixel, const tf::Transform& cameraFrameToWorldFrame,
                             image_geometry::PinholeCameraModel cam)
{
  cv::Point3d cameraRay = cam.projectPixelTo3dRay(pixel);
  tf::Point worldCameraOrigin = cameraFrameToWorldFrame * tf::Vector3(0, 0, 0);
  tf::Point worldCameraStep =
      cameraFrameToWorldFrame * tf::Vector3(cameraRay.x, cameraRay.y, cameraRay.z) - worldCameraOrigin;
  double zScale = -worldCameraOrigin.z() / worldCameraStep.z();
  tf::Point ret = worldCameraOrigin + zScale * worldCameraStep;
  return pcl::PointXYZ(ret.x(), ret.y(), 0);
}

pcl::PointXYZ PointFromPixelNoCam(const cv::Point& p, int height, int width, double HFOV, double VFOV, double origin_z,
                                  double origin_y, double pitch)
{
  int xP = p.x;
  int yP = p.y + (height / 2 - 100);

  double pitch_offset = ((float)(yP - height / 2) / height) * VFOV;
  double y = origin_z / tan(pitch + pitch_offset) + origin_y;

  double theta = ((float)(xP - width / 2) / width) * HFOV;
  double x = y * tan(theta);
  return pcl::PointXYZ(x, y, 0);
}

double toRadians(double degrees)
{
  return degrees / 180.0 * M_PI;
}

std::vector<std::vector<cv::Point>> MatToContours(const cv::Mat img)
{
  std::vector<std::vector<cv::Point>> contours;
  for (int r = img.rows / 2; r < img.rows; r++)
  {
    std::vector<cv::Point> currentCont;
    for (int c = 0; c < img.cols; c++)
    {
      if (img.at<uchar>(r, c) > 0)
      {
        currentCont.push_back(cv::Point(r, c));
      }
    }
    contours.push_back(currentCont);
  }
  return contours;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(tf::TransformListener& tf_listener,
                                                 std::vector<std::vector<cv::Point>> contours,
                                                 image_geometry::PinholeCameraModel cam, std::string topic)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf::StampedTransform transform;
  tf_listener.lookupTransform("/base_footprint", topic, ros::Time(0), transform);

  for (std::vector<cv::Point> contour : contours)
  {
    for (cv::Point p : contour)
    {
      cloud->points.push_back(PointFromPixel(p, transform, cam));
    }
  }

  cloud->header.frame_id = "base_footprint";
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(tf::TransformListener& tf_listener,
                                                 std::vector<std::vector<cv::Point>> contours, int height, int width,
                                                 std::string topic)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf::StampedTransform transform;
  tf_listener.lookupTransform("/base_footprint", topic, ros::Time(0), transform);
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  double origin_z = transform.getOrigin().getZ();
  double origin_y = transform.getOrigin().getY();
  double HFOV = toRadians(66.0);
  double VFOV = toRadians(47.6);
  pitch = -roll;

  for (std::vector<cv::Point> contour : contours)
  {
    for (cv::Point p : contour)
    {
      cloud->points.push_back(PointFromPixelNoCam(p, height, width, HFOV, VFOV, origin_z, origin_y, pitch));
    }
  }

  cloud->header.frame_id = "base_footprint";
  return cloud;
}

#endif  // CVUTILS_H