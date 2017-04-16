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

std::vector<std::vector<cv::Point>> MatToContours(const cv::Mat img)
{
  std::vector<std::vector<cv::Point>> contours;
  for (int r = 0; r < img.rows; r++)
  {
    std::vector<cv::Point> currentCont;
    for (int c = 0; c < img.cols; c++)
    {
      if (img.at<uchar>(r, c) > 0)
      {
        currentCont.push_back(cv::Point(c, r));
      }
    }
    contours.push_back(currentCont);
  }
  return contours;
}

bool replace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

void capPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cap)
{
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for (it = cloud->begin(); it < cloud->end();)
  {
    if (it->x > cap)
    {
      it = cloud->erase(it);
    }
    else
    {
      it++;
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(tf::TransformListener& tf_listener,
                                                 std::vector<std::vector<cv::Point>> contours,
                                                 image_geometry::PinholeCameraModel cam, std::string topic)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  tf::StampedTransform transform;
  std::string topicCopy = topic;
  replace(topicCopy, "usb", "optical");
  tf_listener.lookupTransform("/base_link", topicCopy, ros::Time(0), transform);

  for (std::vector<cv::Point> contour : contours)
  {
    for (cv::Point p : contour)
    {
      cloud->points.push_back(PointFromPixel(p, transform, cam));
    }
  }

  cloud->header.frame_id = "base_link";
  return cloud;
}

#endif  // CVUTILS_H
