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
  //TODO: figure out how to discount bad line models/look into weird artifact with off-center barrels
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

GroundSegmenter::GroundSegmenter(std::vector<velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator<velodyne_pointcloud::PointXYZIR>> points, int nnum_segments, double bbin_width)
{
  num_segments = nnum_segments;
  bin_width = bbin_width;
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
      Prototype ptype = {getDistanceFromPoint(prototype_pt), prototype_pt};
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
        seg.second.lines.emplace_back(curr_line);
        curr_line = Line(error_t);
        curr_line.fitPoints(pt);
      }
    }
    seg.second.lines.emplace_back(curr_line);
  }
}

int GroundSegmenter::getAngleFromPoint(velodyne_pointcloud::PointXYZIR point)
{
  return (int)(atan(point.y / point.x) * 180 / M_PI) % 360;
}

double GroundSegmenter::getDistanceFromPoint(velodyne_pointcloud::PointXYZIR point)
{
  return sqrt(point.x * point.x + point.y * point.y);
}