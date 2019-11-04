#include "GroundSegmenter.h"

bool Line::fitPoints(const std::vector<Prototype> points) {
  int numPoints = points.size();
  if (numPoints < 2) 
  {
    return false;
  }
  
}

GroundSegmenter::GroundSegmenter(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points, int num_segments, int bin_width)
{
  this->num_segments = num_segments;
  this->bin_width = bin_width;
  for (const auto &point : points) 
  {
    int segment_id = getAngleFromPoint(point) * num_segments / 360;
    segments_raw[segment_id].emplace_back(point);
  }
}

void GroundSegmenter::processSegments()
{
  for (const auto &segment : segments_raw) 
  {
    std::map<int, std::vector<pcl::PointXYZ>> bins_raw;
    for (const auto &point : segment.second)
    {
      int bin_id = getDistanceFromPoint(point) / bin_width;
      bins_raw[bin_id].emplace_back(point);
    }
    for (const auto &bin : bins_raw) 
    {
      pcl::PointXYZ prototype_pt = bin.second[0];
      for (const auto &point : bin.second)
      {
        if (point.z < prototype_pt.z)
        {
          prototype_pt = point;
        }
      }
      Prototype ptype = {getDistanceFromPoint(prototype_pt), prototype_pt};
      segments_processed[segment.first].emplace_back(ptype);
    }
  }
}

void GroundSegmenter::getLinesFromSegments()
{
  for (const auto &segment : segments_processed) 
  {
    std::vector<Prototype> sorted_points = segment.second;
    sort(sorted_points.begin(), sorted_points.end());
    //TODO

  }
}

int GroundSegmenter::getAngleFromPoint(pcl::PointXYZ point)
{
  //Maybe sketch?
  return (int)(tan(point.y / point.x) * 180 / M_PI) % 360;
}

int GroundSegmenter::getDistanceFromPoint(pcl::PointXYZ point)
{
  return (int)(sqrt(point.x * point.x + point.y * point.y));
}