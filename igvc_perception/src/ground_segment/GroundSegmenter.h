#ifndef GROUNDSEGMENTER_H
#define GROUNDSEGMENTER_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <gsl/gsl_fit.h>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

struct Prototype
{
  double distance;
  velodyne_pointcloud::PointXYZIR point;

  bool operator<(const Prototype &p) const {return distance < p.distance;}
};

struct Line
{
  Eigen::Vector3d params;
  Eigen::Vector3d intercept;
  Prototype start_point;
  Prototype end_point;
  double error_t;
  std::vector<Prototype> model_points;

  Line(double threshold) {error_t = threshold;}
  double distFromPoint(const Prototype p);
  bool fitPoints(const Prototype new_point);
};

struct Segment
{
  std::vector<velodyne_pointcloud::PointXYZIR> raw_points;
  std::vector<Prototype> prototype_points;
  std::vector<Line> lines;
};

class GroundSegmenter 
{
public:
  GroundSegmenter(std::vector<velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator<velodyne_pointcloud::PointXYZIR>> points, int num_segments, double bin_width);
  
  int num_segments;
  double bin_width;
  std::map<int, Segment> segments;

  void processSegments();
  void getLinesFromSegments(double error_t);

private:
  int getAngleFromPoint(velodyne_pointcloud::PointXYZIR point);
  double getDistanceFromPoint(velodyne_pointcloud::PointXYZIR point);
};

#endif