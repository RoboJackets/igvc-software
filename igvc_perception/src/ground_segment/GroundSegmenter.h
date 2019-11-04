#ifndef GROUNDSEGMENTER_H
#define GROUNDSEGMENTER_H

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <gsl/gsl_fit.h>

struct Prototype
{
  int distance;
  pcl::PointXYZ point;

  bool operator<(const Prototype &p) const {return distance > p.distance;}
};

struct Line
{
  pcl::PointXYZ intercept;
  double slope;

  bool fitPoints(const std::vector<Prototype> points);
};

class GroundSegmenter 
{
public:
  GroundSegmenter(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points, int num_segments, int bin_width);
  
  int num_segments;
  int bin_width;
  std::map<int, std::vector<pcl::PointXYZ>> segments_raw;
  std::map<int, std::vector<Prototype>> segments_processed;

  void processSegments();
  void getLinesFromSegments();

private:
  int getAngleFromPoint(pcl::PointXYZ point);
  int getDistanceFromPoint(pcl::PointXYZ point);
};

#endif