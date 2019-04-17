#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_navigation/signed_distance_field.h>
#include <igvc_utils/NodeUtils.hpp>

#include "ros_trajectory_controller.h"
#include "trajectory_controller.h"

namespace ros_trajectory_controller
{
using differential_drive_model::DifferentialDriveOptions;
using sdf_cost::SDFCostOptions;
using trajectory_controller::ControllerResult;

ROSTrajectoryController::ROSTrajectoryController() : nh{}, pNh{ "~" }, state_{ std::nullopt }
{
  float starting_yaw;
  float cos_scaling;
  igvc::param(pNh, "node/debug", debug_, true);
  igvc::getParam(pNh, "debug/starting_yaw", starting_yaw);
  igvc::getParam(pNh, "debug/cos_scaling", cos_scaling);
  initSubscribeAndPublish();
  initController();
  initMiscParams();

  //    ros::spin();
  ros::Rate rate(1);
  while (ros::ok())
  {
    testController(starting_yaw, cos_scaling);
    rate.sleep();
  }
}

void ROSTrajectoryController::testController(float starting_yaw, float cos_scaling)
{
  RobotState state{};
  state.yaw = starting_yaw;
  state.wheel_velocity_.left = 0.5;
  state.wheel_velocity_.right = 0.5;

  nav_msgs::PathPtr path = boost::make_shared<nav_msgs::Path>();
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path->poses.emplace_back(pose);
  pose.pose.position.x = 5;
  pose.pose.position.y = 0;
  path->poses.emplace_back(pose);
  pose.pose.position.x = 5;
  pose.pose.position.y = 5;
  path->poses.emplace_back(pose);
  std::unique_ptr<ControllerResult> result = controller_->getControls(path, state);
  publishAsPCL(debug_signed_distance_field_pub_, *result->signed_distance_field, sdf_options_->resolution_, "/odom",
               pcl_conversions::toPCL(ros::Time::now()), cos_scaling);
  visualizeRollout(result->optimization_result, ros::Time::now());
}

void ROSTrajectoryController::publishAsPCL(const ros::Publisher& pub, const cv::Mat& mat, double resolution,
                                           const std::string& frame_id, uint64_t stamp, float cos_scaling) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < mat.cols; i++)
  {
    for (int j = 0; j < mat.rows; j++)
    {
      pcl::PointXYZI p{};
      p.x = static_cast<float>((i - (mat.cols / 2.0)) * resolution) + state_->x;
      p.y = static_cast<float>(((mat.rows / 2.0) - j) * resolution) + state_->y;
      p.intensity = cos(mat.at<float>(j, i) / cos_scaling);
      pointcloud->points.push_back(p);
    }
  }
  pointcloud->header.frame_id = frame_id;
  pointcloud->header.stamp = stamp;
  pub.publish(pointcloud);
}

void ROSTrajectoryController::initSubscribeAndPublish()
{
  std::string topic_path;
  std::string topic_odometry;
  std::string topic_wheel_odometry;
  std::string topic_motors;

  igvc::getParam(pNh, "topics/path", topic_path);
  igvc::getParam(pNh, "topics/odometry", topic_odometry);
  igvc::getParam(pNh, "topics/wheel_odometry", topic_wheel_odometry);
  igvc::getParam(pNh, "topics/motors", topic_motors);

  motor_pub_ = nh.advertise<igvc_msgs::velocity_pair>(topic_motors, 1);

  path_sub_ = nh.subscribe(topic_path, 1, &ROSTrajectoryController::pathCallback, this);
  odom_sub_ = nh.subscribe(topic_odometry, 1, &ROSTrajectoryController::odomCallback, this);
  wheel_odom_sub_ = nh.subscribe(topic_wheel_odometry, 1, &ROSTrajectoryController::wheelOdomCallback, this);

  if (debug_)
  {
    std::string topic_debug_rollout;
    std::string topic_debug_signed_distance_field;

    igvc::getParam(pNh, "topics/debug/rollout", topic_debug_rollout);
    igvc::getParam(pNh, "topics/debug/signed_distance_field", topic_debug_signed_distance_field);

    debug_rollout_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_debug_rollout, 1);
    debug_signed_distance_field_pub_ =
        nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic_debug_signed_distance_field, 1);
  }
}

void ROSTrajectoryController::initController()
{
  float width;
  float height;
  float resolution;
  float traversal_cost;
  float path_cost;
  bool use_path_cost;

  SomeControllerOptions some_controller_options{};
  SDFCostOptions sdf_cost_options{};
  DifferentialDriveOptions differential_drive_options{};

  igvc::getParam(pNh, "signed_distance_field/width", width);
  igvc::getParam(pNh, "signed_distance_field/height", height);
  igvc::getParam(pNh, "signed_distance_field/resolution", resolution);
  igvc::getParam(pNh, "signed_distance_field/traversal_cost", traversal_cost);
  igvc::getParam(pNh, "signed_distance_field/path_cost", path_cost);
  igvc::getParam(pNh, "signed_distance_field/use_path_cost", use_path_cost);

  igvc::getParam(pNh, "controller/timestep", some_controller_options.timestep);
  igvc::getParam(pNh, "controller/horizon", some_controller_options.horizon);
  igvc::getParam(pNh, "controller/samples", some_controller_options.num_samples);
  igvc::getParam(pNh, "controller/resample_threshold", some_controller_options.resample_threshold);
  igvc::getParam(pNh, "controller/use_weighted", some_controller_options.use_weighted);

  igvc::getParam(pNh, "cost_function/max_velocity", sdf_cost_options.velocity_limit);
  igvc::getParam(pNh, "cost_function/coefficients/path", sdf_cost_options.coefficients.path);
  igvc::getParam(pNh, "cost_function/coefficients/velocity", sdf_cost_options.coefficients.velocity);
  igvc::getParam(pNh, "cost_function/coefficients/acceleration", sdf_cost_options.coefficients.acceleration);
  igvc::getParam(pNh, "cost_function/coefficients/angular_acceleration",
                 sdf_cost_options.coefficients.angular_acceleration);
  igvc::getParam(pNh, "cost_function/coefficients/angular_velocity",
    sdf_cost_options.coefficients.angular_velocity);

  igvc::getParam(pNh, "model/acceleration_bound/lower", differential_drive_options.acceleration_bound.lower);
  igvc::getParam(pNh, "model/acceleration_bound/upper", differential_drive_options.acceleration_bound.upper);
  igvc::getParam(pNh, "model/axle_length", differential_drive_options.axle_length);

  sdf_options_ =
      std::make_unique<SignedDistanceFieldOptions>(width, height, resolution, traversal_cost, path_cost, use_path_cost);
  controller_ = std::make_unique<TrajectoryController>(*sdf_options_, some_controller_options,
                                                       differential_drive_options, sdf_cost_options);
}

void ROSTrajectoryController::pathCallback(const nav_msgs::PathConstPtr& path)
{
  path_ = path;
  getControls(path->header.stamp);
}

void ROSTrajectoryController::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  if (!state_)
  {
    state_ = RobotState(odom);
  }
  else
  {
    state_->setState(odom);
  }
  getControls(odom->header.stamp);
}

void ROSTrajectoryController::wheelOdomCallback(const igvc_msgs::velocity_pair& velocity_pair)
{
  if (!state_)
  {
    state_ = RobotState(velocity_pair);
  }
  else
  {
    state_->setState(velocity_pair);
  }
  getControls(velocity_pair.header.stamp);
}

void ROSTrajectoryController::getControls(const ros::Time& stamp)
{
  if (path_.get() == nullptr)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(1, "ROSTrajectoryController::getControls:path_null", "Path is null");
    return;
  }
  if (!state_)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(1, "ROSTrajectoryController::getControls:state_null", "State is null");
    return;
  }
  std::unique_ptr<ControllerResult> controller_results = controller_->getControls(path_, *state_);

  if (debug_)
  {
    visualizeRollout(controller_results->optimization_result, stamp);
    visualizeSignedDistanceField(controller_results->signed_distance_field, stamp);
  }

  executeControls(controller_results->controls, stamp);
}

void ROSTrajectoryController::executeControls(igvc_msgs::velocity_pair controls, const ros::Time& stamp)
{
  controls.header.stamp = stamp;
  motor_pub_.publish(controls);
}

void ROSTrajectoryController::visualizeRollout(const std::unique_ptr<OptimizationResult<Model>>& optimization_result,
                                               const ros::Time& stamp) const
{
  std::vector<Particle<Model>> particles = optimization_result->particles;
  auto optimal_particle =
      std::min_element(particles.begin(), particles.end(), [](const Particle<Model>& p1, const Particle<Model>& p2) {
        return p1.cum_cost_.back() < p2.cum_cost_.back();
      });

  float max_weight = optimal_particle->getWeight();

  int id = 0;
  visualization_msgs::MarkerArray marker_array;
  for (const Particle<Model>& particle : optimization_result->particles)
  {
    //    ROS_INFO_STREAM("optimal particle cost: " << optimal_particle->cum_cost_.back() << ", particle weight: " <<
    //    particle.getWeight() << ", max_weight: " << max_weight << ", ratio: " << particle.getWeight() / max_weight);
    marker_array.markers.emplace_back(toLineStrip(particle.state_vec_, id++, sampled_width_,
                                                  1.0f - particle.getWeight() / max_weight,
                                                  particle.getWeight() / max_weight, 0.0f, particle.getWeight() / max_weight, stamp));
  }
  Particle<Model> best_particle = optimization_result->weighted_particle;
  marker_array.markers.emplace_back(
      toLineStrip(best_particle.state_vec_, id++, weighted_width_, 1.0f, 1.0f, 1.0f, 0.8f, stamp));
  debug_rollout_pub_.publish(marker_array);
}

void ROSTrajectoryController::visualizeSignedDistanceField(const std::unique_ptr<cv::Mat>& signed_distance_field,
                                                           const ros::Time& stamp) const
{
  publishAsPCL(debug_signed_distance_field_pub_, *signed_distance_field, sdf_options_->resolution_, "/odom",
               pcl_conversions::toPCL(stamp));
}

visualization_msgs::Marker ROSTrajectoryController::toLineStrip(const std::vector<State>& states, int id, float width,
                                                                float r, float g, float b, float a,
                                                                const ros::Time& stamp) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.header.stamp = stamp;

  marker.ns = "rollout";
  marker.id = id;

  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (const State& state : states)
  {
    geometry_msgs::Point point;
    point.x = state.x;
    point.y = state.y;
    marker.points.emplace_back(point);
  }
  return marker;
}

void ROSTrajectoryController::initMiscParams()
{
  igvc::param(pNh, "visualization/sampled_width", sampled_width_, 0.0005);
  igvc::param(pNh, "visualization/weighted_width", weighted_width_, 0.001);
}
}  // namespace ros_trajectory_controller
