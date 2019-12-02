#include <pointcloud_filter/fast_segment_filter/fast_segment_filter.h>

namespace pointcloud_filter
{
void FastSegmentFilter::filter(pointcloud_filter::Bundle &bundle)
{
  segments_.clear();
  ground_points_.clear();
  nonground_points_.clear();

  for (const auto &point : bundle.pointcloud->points)
  {
    int segment_id = getSegIdFromPoint(point);
    segments_[segment_id].raw_points_.emplace_back(point);
  }
  computePrototypePoints();
  getLinesFromSegments();
  if (config_.debug_viz)
  {
    debugViz();
  }
  ground_points_.header.frame_id = "/lidar";
  nonground_points_.header.frame_id = "/lidar";
  classifyPoints(ground_points_, nonground_points_);
  ground_pub_.publish(ground_points_);
  nonground_pub_.publish(nonground_points_);

  bundle.occupied_pointcloud->points = std::move(nonground_points_.points);
}

double Line::distFromPoint(const Prototype point) const
{
  Eigen::Vector3d vect;
  vect << point.point_.x, point.point_.y, point.point_.z;
  double t = -(intercept_ - vect).dot(slope_) / (slope_.squaredNorm());
  Eigen::Vector3d pt_on_line = t * slope_ + intercept_;
  return (pt_on_line - vect).squaredNorm();
}

bool Line::attemptFitPoint(const Prototype &new_point)
{
  int total_points = model_points_.size() + 1;
  if (total_points < 3)
  {
    if (model_points_.empty())
    {
      start_point_ = new_point;
    }
    else
    {
      end_point_ = new_point;
    }
    model_points_.emplace_back(new_point);
    return true;
  }
  Eigen::Vector3d old_params = slope_;
  Eigen::Vector3d old_intercept = intercept_;
  Eigen::MatrixXd A(total_points, 3);
  Eigen::MatrixXd mean(1, 3);
  for (int i = 0; i < total_points; i++)
  {
    Prototype pt;
    if (i == total_points - 1)
    {
      pt = new_point;
    }
    else
    {
      pt = model_points_[i];
    }

    A(i, 0) = pt.point_.x;
    A(i, 1) = pt.point_.y;
    A(i, 2) = pt.point_.z;
    mean(0, 0) += pt.point_.x;
    mean(0, 1) += pt.point_.y;
    mean(0, 2) += pt.point_.z;
  }
  mean /= -total_points;
  A += mean.replicate(total_points, 1);
  getSlopeIntercept(A, mean);
  double squared_error = distFromPoint(new_point);
  for (const Prototype &pt : model_points_)
  {
    squared_error += distFromPoint(pt);
  }
  if (squared_error > error_t_)
  {
    slope_ = old_params;
    intercept_ = old_intercept;
    return false;
  }
  model_points_.emplace_back(new_point);
  end_point_ = new_point;
  return true;
}

void Line::getSlopeIntercept(const Eigen::MatrixXd &A, const Eigen::MatrixXd &mean)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  slope_ = svd.matrixV().col(0);
  intercept_(0) = -mean(0, 0);
  intercept_(1) = -mean(0, 1);
  intercept_(2) = -mean(0, 2);
  slope_.normalize();
}

FastSegmentFilter::FastSegmentFilter(const ros::NodeHandle &nh) : private_nh_{ nh }, config_{ nh }
{
  ground_pub_ = private_nh_.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(config_.ground_topic, 1);
  nonground_pub_ = private_nh_.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>>(config_.nonground_topic, 1);
  marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("Lines_array", 1);
  ;
}

void FastSegmentFilter::computePrototypePoints()
{
  for (const auto &seg : segments_)
  {
    std::map<int, std::vector<velodyne_pointcloud::PointXYZIR>> bins_raw;
    for (const auto &point : seg.second.raw_points_)
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
      Prototype ptype = { getDistanceFromPoint(prototype_pt), prototype_pt };
      segments_[seg.first].prototype_points_.emplace_back(ptype);
    }
  }
}

void FastSegmentFilter::getLinesFromSegments()
{
  for (auto &seg : segments_)
  {
    sort(seg.second.prototype_points_.begin(), seg.second.prototype_points_.end());
    Line curr_line = Line(config_.error_t);
    for (const Prototype &pt : seg.second.prototype_points_)
    {
      if (!curr_line.attemptFitPoint(pt))
      {
        curr_line.is_ground_ = evaluateIsGround(curr_line);
        seg.second.lines_.emplace_back(curr_line);
        curr_line = Line(config_.error_t);
        curr_line.attemptFitPoint(pt);
      }
    }
    curr_line.is_ground_ = evaluateIsGround(curr_line);
    if (curr_line.model_points_.size() > 1)
    {
      seg.second.lines_.emplace_back(curr_line);
    }
  }
}

void FastSegmentFilter::classifyPoints(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points,
                                       pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &nonground_points)
{
  for (const auto &[segment_id, segment] : segments_)
  {
    for (const auto &point : segment.raw_points_)
    {
      Line mapped_line;
      double min_dist = -1;
      for (const auto &line : segment.lines_)
      {
        for (const auto &pt : line.model_points_)
        {
          double distance = getDistanceBetweenPoints(point, pt.point_);
          if (min_dist == -1 || distance < min_dist)
          {
            min_dist = distance;
            mapped_line = line;
          }
        }
      }
      if (mapped_line.is_ground_)  // Removed max dist = nonground since was more pain than good
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

int FastSegmentFilter::getSegIdFromPoint(const velodyne_pointcloud::PointXYZIR point)
{
  double angle = atan2(point.y, point.x) + M_PI;
  return angle * config_.num_segments / (2 * M_PI);
}

double FastSegmentFilter::getDistanceFromPoint(const velodyne_pointcloud::PointXYZIR point)
{
  return std::hypot(point.x, point.y);
}

double FastSegmentFilter::getDistanceBetweenPoints(const velodyne_pointcloud::PointXYZIR point1,
                                                   const velodyne_pointcloud::PointXYZIR point2)
{
  return std::hypot(point1.x - point2.x, point1.y - point2.y, point1.z - point2.z);
}

bool FastSegmentFilter::evaluateIsGround(Line &l)
{
  const auto above_intercept = [&](const Prototype &pt) { return pt.point_.z > config_.dist_t; };
  bool all_below = std::all_of(l.model_points_.begin(), l.model_points_.end(), above_intercept);
  if (all_below)
  {
    return true;
  }
  double dx = l.end_point_.point_.x - l.start_point_.point_.x;
  double dy = l.end_point_.point_.y - l.start_point_.point_.y;
  double dz = l.end_point_.point_.z - l.start_point_.point_.z;
  double len_xy = std::hypot(dx, dy);
  double slope = std::abs(dz) / len_xy;
  return slope < config_.slope_t && l.end_point_.point_.z < config_.intercept_z_t;
}

void FastSegmentFilter::debugViz()
{
  visualization_msgs::MarkerArray lines;
  int i = 0;

  for (const auto &seg : segments_)
  {
    for (const Line &line : seg.second.lines_)
    {
      for (Prototype pt : line.model_points_)
      {
        visualization_msgs::Marker points;
        points.header.frame_id = "/lidar";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = i++;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        if (line.is_ground_)
        {
          points.color.b = 1.0;
        }
        else
        {
          points.color.r = 1.0;
        }
        points.color.a = 1.0;
        geometry_msgs::Point p;
        p.x = pt.point_.x;
        p.y = pt.point_.y;
        p.z = pt.point_.z;
        points.points.push_back(p);
        lines.markers.push_back(points);
      }
      visualization_msgs::Marker line_list;
      line_list.header.frame_id = "/lidar";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "lines";
      line_list.id = i++;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.scale.x = 0.025;
      if (line.is_ground_)
      {
        line_list.color.b = 1.0;
      }
      else
      {
        line_list.color.r = 1.0;
      }
      line_list.color.a = 1.0;
      geometry_msgs::Point p1;
      p1.x = line.start_point_.point_.x;
      p1.y = line.start_point_.point_.y;
      p1.z = line.start_point_.point_.z;
      line_list.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = line.end_point_.point_.x;
      p2.y = line.end_point_.point_.y;
      p2.z = line.end_point_.point_.z;
      line_list.points.push_back(p2);
      lines.markers.push_back(line_list);
    }
  }
  marker_pub_.publish(lines);
}
}  // namespace pointcloud_filter
