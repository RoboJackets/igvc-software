#ifndef GROUNDSEGMENTER_H
#define GROUNDSEGMENTER_H

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_filter/fast_segment_filter/fast_segment_filter_config.h>
#include <pointcloud_filter/filter.h>
#include <velodyne_pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace pointcloud_filter
{
/**
 * A class that represents a prototype point, where a prototype
 * point is a point that will be used to model lines within segments.
 * There is one prototype point per bin, and since bins are determined
 * by lidar rings, the number of lidar rings determines the number of prototype
 * points per segment.
 */
struct Prototype
{
  double distance_ = 0.0;
  velodyne_pcl::PointXYZIRT point_ = {};

  bool operator<(const Prototype &p) const
  {
    return distance_ < p.distance_;
  }
};

/**
 * A struct to represent a line over the form:
 *   slope_ * t + intercept_
 * where t is a parameter.
 * The slope and intercept define the line. The slope is defined from a least squares approach using SVD.
 * And the intercept is defined as the mean of the points used to estimate the slope.
 */

struct Line
{
  Eigen::Vector3d slope_ = {};
  Eigen::Vector3d intercept_ = {};
  Prototype start_point_ = {};
  Prototype end_point_ = {};
  double error_t_ = 0.0;
  std::vector<Prototype> model_points_ = {};
  bool is_ground_ = false;

  Line() = default;
  Line(double threshold) : error_t_(threshold)
  {
  }

  double distFromPoint(const Prototype p) const;

  /**
   * Considers new_point in addition to existing model_points, it then
   * creates a model of a line using a least squares approach given these points.
   * If the new model has too much error, new_point is not added to model_points
   * and attemptFitPoint returns false.
   * Else new_point is added to model_points and attemptFitPoint returns true.
   *
   * @param new_point A new point to be used with model_points to estimate a line
   * @return True if new_point + model_points create a line without too much error
   *         False otherwise
   *
   */
  bool attemptFitPoint(const Prototype &new_point);

  /**
   * Computes a slope and intercept for the line given a set of points. Uses SVD
   * to compute the slope and uses the mean of the points as the intercept.
   *
   * @param A Matrix representing the sample points to fit the line/get slope from
   * @param mean Matrix that is a 3d point which is the mean of all the sample points
   *
   */
  void getSlopeIntercept(const Eigen::MatrixXd &A, const Eigen::MatrixXd &mean);
};

struct Segment
{
  std::vector<velodyne_pcl::PointXYZIRT> raw_points_;
  std::vector<Prototype> prototype_points_;
  std::vector<Line> lines_;
};

class FastSegmentFilter
{
public:
  FastSegmentFilter(const ros::NodeHandle &nh);

  std::unordered_map<int, Segment> segments_;
  pcl::PointCloud<velodyne_pcl::PointXYZIRT> ground_points_;
  pcl::PointCloud<velodyne_pcl::PointXYZIRT> nonground_points_;

  ros::Publisher ground_pub_;
  ros::Publisher nonground_pub_;
  ros::Publisher marker_pub_;
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string velodyne_topic_;

  ros::NodeHandle private_nh_;

  void filter(pointcloud_filter::Bundle &bundle);

  /**
   * This function goes through each segment and for each:
   *   -Split the points into bins based on lidar ring.
   *   -For each bin, pick one point to be the prototype point, to be used for
   *    fitting lines within the segment.
   */
  void computePrototypePoints();

  /**
   * This goes through each segment and creates lines from the prototype points
   * to be used to determine ground versus nonground.
   * For each segment, this creates a new line with no model_points yet
   * and iterates through the prototype points one-by-one.
   * It then calls attemptFitPoint on each point.
   *   -If attemptFitPoint returns true, than the line model is still valid and the point was added
   *   -If attemptFitPoint returns false, than the line model had too much error when considered the
   *    current point and the point was not added. The old line is added to the segment's list
   *    of lines, and a new line is created with its model_points only containing the current point.
   */
  void getLinesFromSegments();

  /**
   * This function goes through all raw_points in each segment and classifies each
   * point as ground or nonground based on whether the closest line to it is
   * ground or nonground
   *
   * @param ground_points Output parameter for determined ground points
   * @param nonground_points Output parameter for determined nonground points
   *
   */

  void classifyPoints(pcl::PointCloud<velodyne_pcl::PointXYZIRT> &ground_points,
                      pcl::PointCloud<velodyne_pcl::PointXYZIRT> &nonground_points);
  void debugViz();

private:
  FastSegmentFilterConfig config_{};

  int getSegIdFromPoint(const velodyne_pcl::PointXYZIRT point);
  double getDistanceFromPoint(const velodyne_pcl::PointXYZIRT point);
  double getDistanceBetweenPoints(const velodyne_pcl::PointXYZIRT point1, const velodyne_pcl::PointXYZIRT point2);
  bool evaluateIsGround(Line &l);
};
}  // namespace pointcloud_filter

#endif