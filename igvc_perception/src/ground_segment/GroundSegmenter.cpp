#include "GroundSegmenter.h"

double Line::distFromPoint(const Prototype point)
{
  Eigen::Vector3d vect;
  vect(0) = point.point.x;
  vect(1) = point.point.y;
  vect(2) = point.point.z;
  double t = -(intercept - vect).dot(params) / (params.dot(params));
  Eigen::Vector3d pt_on_line = t * params + intercept;
  return (pt_on_line - vect).squaredNorm();
}

bool Line::fitPoints(const Prototype new_point) 
{
  int total_points = model_points.size() + 1;
  if (total_points < 2) 
  {
    model_points.emplace_back(new_point);
    start_point = new_point;
    return true;
  } 
  Eigen::Vector3d old_params = params;
  Eigen::Vector3d old_intercept = intercept;
  MatrixXd A(total_points, 3);
  MatrixXd mean(1, 3);
  for (int i = 0; i < total_points; i++) 
  {
    Prototype pt;
    if (i == total_points - 1)
    {
      pt = new_point;
    }
    else
    {
      pt = model_points[i];      
    }
    
    A(i, 0) = pt.point.x;
    A(i, 1) = pt.point.y;
    A(i, 2) = pt.point.z;
    mean(0, 0) += pt.point.x;
    mean(0, 1) += pt.point.y;
    mean(0, 2) += pt.point.z;
  }
  mean /= -total_points;
  A += mean.replicate(total_points, 1);
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  params = svd.matrixV().col(0);
  intercept(0) = -mean(0, 0);
  intercept(1) = -mean(0, 1);
  intercept(2) = -mean(0, 2);
  params.normalize();
  double squared_error = distFromPoint(new_point);
  for (Prototype pt : model_points)
  {
    squared_error += distFromPoint(pt);
  }
  if (squared_error > error_t)
  {
    params = old_params;
    intercept = old_intercept;
    return false;
  }
  model_points.emplace_back(new_point);
  end_point = new_point;
  return true;
}

GroundSegmenter::GroundSegmenter(std::vector<velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator<velodyne_pointcloud::PointXYZIR>> points, int p_num_segments, double p_slope_t, double p_intercept_z_t, double p_dist_t)
{
  num_segments = p_num_segments;
  slope_t = p_slope_t;
  intercept_z_t = p_intercept_z_t;
  dist_t = p_dist_t;
  for (const auto &point : points) 
  {
    int segment_id = getAngleFromPoint(point) * num_segments / 360;
    segments[segment_id].raw_points.emplace_back(point);
  }
}

void GroundSegmenter::processSegments()
{
  for (const auto &seg : segments) 
  {
    std::map<int, std::vector<velodyne_pointcloud::PointXYZIR>> bins_raw;
    for (const auto &point : seg.second.raw_points)
    {
      bins_raw[point.ring].emplace_back(point);
    }
    for (const auto &bin : bins_raw) 
    {
      velodyne_pointcloud::PointXYZIR prototype_pt = bin.second[0];
      for (const auto &point : bin.second)
      {
        if (point.z < prototype_pt.z)
        {
          prototype_pt = point;
        }
      }
      Prototype ptype = {getDistanceFromPoint(prototype_pt), prototype_pt, 0, 0, 0};
      segments[seg.first].prototype_points.emplace_back(ptype);
    }
  }
}

void GroundSegmenter::getLinesFromSegments(double error_t)
{
  for (auto &seg : segments) 
  {
    std::vector<Prototype> sorted_points = seg.second.prototype_points;
    sort(sorted_points.begin(), sorted_points.end());
    Line curr_line = Line(error_t);
    for (Prototype pt : sorted_points) {
      if (!curr_line.fitPoints(pt)) {
        curr_line.isGround = evaluateIsGround(curr_line);
        if (curr_line.isGround)
        {
          curr_line.blue = 1.0;
          curr_line.red = 0.0;
        }
        else 
        {
          curr_line.red = 1.0;
          curr_line.blue = 0.0;
        }
        for (Prototype &pt : curr_line.model_points)
        {
          pt.blue = curr_line.blue;
          pt.green = curr_line.green;
          pt.red = curr_line.red;
        }
        seg.second.lines.emplace_back(curr_line);
        curr_line = Line(error_t);
        curr_line.fitPoints(pt);
      }
    }
    curr_line.isGround = evaluateIsGround(curr_line);
    if (curr_line.isGround)
    {
      curr_line.blue = 1.0;
      curr_line.red = 0.0;
    }
    else 
    {
      curr_line.red = 1.0;
      curr_line.blue = 0.0;
    }
    for (Prototype &pt : curr_line.model_points)
    {
      pt.blue = curr_line.blue;
      pt.green = curr_line.green;
      pt.red = curr_line.red;
    }
    seg.second.lines.emplace_back(curr_line);
  }
}

void GroundSegmenter::classifyPoints(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &nonground_points)
{
  for (const auto &seg : segments)
  {
    Segment segment = seg.second;
    for (const auto &point : segment.raw_points)
    {
      Line mapped_line;
      double min_dist = -1;
      for (const auto &line : segment.lines)
      {
        for (const auto &pt : line.model_points)
        {
          double distance = getDistanceBetweenPoints(point, pt.point);
          if (min_dist == -1 || distance < min_dist)
          {
            min_dist = distance;
            mapped_line = line;
          }
        }
      }
      if (mapped_line.isGround) //&& min_dist < dist_t)
      {
        ground_points.push_back(point);
      }
      else
      {
        nonground_points.push_back(point);
      }
    }
  }
}

int GroundSegmenter::getAngleFromPoint(const velodyne_pointcloud::PointXYZIR point)
{
  return (int)(atan(point.y / point.x) * 180 / M_PI) % 360;
}

double GroundSegmenter::getDistanceFromPoint(const velodyne_pointcloud::PointXYZIR point)
{
  return sqrt(point.x * point.x + point.y * point.y);
}

double GroundSegmenter::getDistanceBetweenPoints(const velodyne_pointcloud::PointXYZIR point1, const velodyne_pointcloud::PointXYZIR point2)
{
  return pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2);
}

bool GroundSegmenter::evaluateIsGround(const Line l)
{
  double slope = abs(l.end_point.point.z - l.start_point.point.z) / sqrt(pow(l.end_point.point.y - l.start_point.point.y, 2) + pow(l.end_point.point.x - l.start_point.point.x, 2));
  return slope < slope_t && l.end_point.point.z < intercept_z_t;
}