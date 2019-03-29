#include "mapper.h"

void Mapper::insertLidarScan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
{

}

void Mapper::insertCameraProjection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
{

}

void Mapper::insertSegmentedImage(const sensor_msgs::ImageConstPtr& image)
{

}

std::shared_ptr<cv::Mat> Mapper::getMap()
{
  return std::shared_ptr<cv::Mat>();
}
