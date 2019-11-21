#ifndef GROUNDSEGMENTER_H
#define GROUNDSEGMENTER_H

#include <gsl/gsl_fit.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_filter/fast_segment_filter/fast_segment_filter_config.h>
#include <pointcloud_filter/filter.h>
#include <velodyne_pointcloud/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

namespace pointcloud_filter
{
struct Prototype
{
  double distance_;
  velodyne_pointcloud::PointXYZIR point_;
  double blue_;
  double red_;
  double green_;

  bool operator<(const Prototype &p) const
  {
    return distance_ < p.distance_;
  }
};

struct Line
{
  Eigen::Vector3d params_;
  Eigen::Vector3d intercept_;
  Prototype start_point_;
  Prototype end_point_;
  double error_t_;
  std::vector<Prototype> model_points_;
  bool isGround_;
  double blue_;
  double red_;
  double green_;

  Line()
  {
  }
  Line(double threshold)
  {
    error_t_ = threshold;
  }
  double distFromPoint(const Prototype p);
  bool fitPoints(const Prototype new_point);
};

struct Segment
{
  std::vector<velodyne_pointcloud::PointXYZIR> raw_points_;
  std::vector<Prototype> prototype_points_;
  std::vector<Line> lines_;
};

class FastSegmentFilter
{
public:
  FastSegmentFilter(const ros::NodeHandle &nh);

  std::map<int, Segment> segments_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ground_points_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> nonground_points_;

  ros::Publisher ground_pub_;
  ros::Publisher nonground_pub_;
  ros::Publisher marker_pub_;
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string velodyne_topic_;

  ros::NodeHandle private_nh_;

  void filter(pointcloud_filter::Bundle &bundle);
  void processSegments();
  void getLinesFromSegments();
  void classifyPoints(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &ground_points,
                      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &nonground_points);
  void debugViz();

private:
  FastSegmentFilterConfig config_{};

  int getAngleFromPoint(const velodyne_pointcloud::PointXYZIR point);
  double getDistanceFromPoint(const velodyne_pointcloud::PointXYZIR point);
  double getDistanceBetweenPoints(const velodyne_pointcloud::PointXYZIR point1,
                                  const velodyne_pointcloud::PointXYZIR point2);
  bool evaluateIsGround(Line &l);
};
}  // namespace pointcloud_filter

#endif