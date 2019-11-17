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
  double blue;
  double red;
  double green;

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
  bool isGround;
  double blue;
  double red;
  double green;

  Line() {}
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
  GroundSegmenter(std::vector<velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator<velodyne_pointcloud::PointXYZIR>> points, int num_segments, double slope_t, double intercept_z_t, double dist_t);
  
  int num_segments;
  std::map<int, Segment> segments;
  double slope_t;
  double intercept_z_t;
  double dist_t;
  
  void processSegments();
  void getLinesFromSegments(double error_t);
  void classifyPoints(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &nonground_points);

private:
  int getAngleFromPoint(const velodyne_pointcloud::PointXYZIR point);
  double getDistanceFromPoint(const velodyne_pointcloud::PointXYZIR point);
  double getDistanceBetweenPoints(const velodyne_pointcloud::PointXYZIR point1, const velodyne_pointcloud::PointXYZIR point2);
  bool evaluateIsGround(const Line l);
};

#endif