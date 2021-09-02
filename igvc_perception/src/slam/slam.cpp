#include <slam/slam.h>

// Using statements
using gtsam::symbol_shorthand::B;  // bias measurements
using gtsam::symbol_shorthand::V;  // velocity measurements
using gtsam::symbol_shorthand::X;  // pose measurements

/**
 * Initializes the factor graph and all the subscribers and publishers
 */
Slam::Slam() : pnh_{ "~" }
{
  imu_sub_ = pnh_.subscribe("/imu", 100, &Slam::imuCallback, this);
  gps_sub_ = pnh_.subscribe("/fix", 1, &Slam::gpsCallback, this);
  wheel_sub_ = pnh_.subscribe("/wheel_odometry", 1, &Slam::wheelOdomCallback, this);
  mag_sub_ = pnh_.subscribe("/magnetometer_mag", 100, &Slam::magCallback, this);

  location_pub_ = pnh_.advertise<nav_msgs::Odometry>("/slam/position", 1);
  gps_location_pub_ = pnh_.advertise<nav_msgs::Odometry>("/odometry/gps", 1);

  curr_index_ = 0;
  imu_connected_ = false;
  imu_update_available_ = false;

  initializeDirectionOfLocalMagField();
  initializeNoiseMatrices();
  initializeImuParams();
  initializePriors();
}

/**
 * The gpsCallback adds GPS measurements to the factor graph, publishes GPS odom measurements.
 * Also adds magnetometer factors, wheel odom measurements, and imu factors, then
 * @param gps A NavSatFix message derived from GPS measurements (published by /fix)
 */
void Slam::gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps)
{
  // get the starting location as the origin
  double lat, lon, h, E, N, U;
  lat = gps->latitude;
  lon = gps->longitude;
  h = gps->altitude;
#if defined(_DEBUG)
  ROS_INFO_STREAM("gps measurement number: " << gps->header.seq);
#endif
  if (firstReading)
  {
    origin_ENU = GeographicLib::LocalCartesian(lat, lon, h, GeographicLib::Geocentric::WGS84());
    firstReading = false;
  }
  origin_ENU.Forward(lat, lon, h, E, N, U);

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = E;
  odom.pose.pose.position.y = N;
  odom.pose.pose.position.z = U;
  gps_location_pub_.publish(odom);

  gtsam::GPSFactor gps_factor(X(curr_index_ + 1), gtsam::Point3(E, N, U), gps_noise_);
  graph_.add(gps_factor);

  addMagFactor();
  addWheelOdomFactor();

  if (imu_update_available_)
  {
    integrateAndAddIMUFactor();
    imu_update_available_ = false;
  }
}

/**
 * The imuCallback adds IMU measurements to the accumulator when it receives an IMU measurement
 * @param msg An imu sensor message (published by yostlab_driver_node)
 */
void Slam::imuCallback(const sensor_msgs::Imu &msg)
{
  ros::Time curr_time = ros::Time::now();
  Vec3 measured_acc = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  Vec3 measured_omega = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

  if (!imu_connected_)
  {
    imu_connected_ = true;
    accum_.integrateMeasurement(measured_acc, measured_omega, 0.005);
  }
  else
  {
    double deltaT = (curr_time - last_imu_measurement_).toSec();
    if (deltaT > 0)
      accum_.integrateMeasurement(measured_acc, measured_omega, deltaT);
  }
  imu_update_available_ = true;
  last_imu_measurement_ = curr_time;
}

/**
 * The magCallback updates the curr_mag_reading_ with its most recent value
 * @param msg An MagneticField sensor message (published by yostlab_driver_node)
 */
void Slam::magCallback(const sensor_msgs::MagneticField &msg)
{
  curr_mag_reading_ = gtsam::Point3(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
}

/**
 * Adds the magFactor to the factor graph
 */
void Slam::addMagFactor()
{
  MagPoseFactor mag_factor(X(curr_index_ + 1), curr_mag_reading_, scale_, local_mag_field_,
                           gtsam::Point3(1e-9, 1e-9, 1e-9), mag_noise_);
  graph_.add(mag_factor);
}

/**
 * The wheelOdomCallback updates the curr_mag_reading_ with its most recent value
 * @param msg An odometry sensor message
 */
void Slam::wheelOdomCallback(const nav_msgs::Odometry &msg)
{
  if (g_last_time.sec == 0)
  {
    g_last_time = ros::Time::now();
  }
  ros::Duration delta_t = msg.header.stamp - g_last_time;
  double dt = delta_t.toSec();

  // the local frame velocities
  double vx = msg.twist.twist.linear.x;
  double vy = msg.twist.twist.linear.y;  // currently zero with jessi but that will change with swervi

  // update the relative position from the previous factor
  g_x += vx * dt * cos(g_theta) - vy * dt * sin(g_theta);
  g_y += vx * dt * sin(g_theta) + vy * dt * cos(g_theta);

  g_theta += dt * msg.twist.twist.angular.z;
  g_xVar += dt * msg.twist.covariance[0];
  g_yVar += dt * msg.twist.covariance[7];
  g_zVar += dt * msg.twist.covariance[14];
  g_thetaVariance += dt * msg.twist.covariance[35];

  g_last_time = msg.header.stamp;
#if defined(_DEBUG)
  ROS_INFO_STREAM("Wheel odom reading: " << curr_wheelOdom_reading_.x() << ", " << curr_wheelOdom_reading_.y() << ", "
                                         << curr_wheelOdom_reading_.theta());
#endif
}

/**
 * Adds the wheelOdom measurement to the factor graph
 */
void Slam::addWheelOdomFactor()
{
  gtsam::Pose3 betweenPose(gtsam::Rot3::Rz(g_theta), gtsam::Point3(g_x, g_y, 0.0));

  auto factor = gtsam::BetweenFactor<gtsam::Pose3>(
      X(curr_index_), X(curr_index_ + 1), betweenPose,
      noiseDiagonal::Sigmas(
          (gtsam::Vector(6) << g_thetaVariance * 2, g_thetaVariance * 2, g_thetaVariance, g_xVar, g_yVar, g_zVar)
              .finished()));

  graph_.add(factor);
  
  g_x = 0;
  g_y = 0;
  g_theta = 0;
  g_xVar = 0;
  g_yVar = 0;
  g_zVar = 0;
  g_thetaVariance = 0;
}

/**
 * If there are IMU measurements in the accumulator, this adds them as a single factor to the factor graph and optimizes
 * graph.
 */
void Slam::integrateAndAddIMUFactor()
{
  if (accum_.preintMeasCov().trace() != 0)
  {
    // Add bias factor
    auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(
        B(curr_index_), B(curr_index_ + 1), gtsam::imuBias::ConstantBias(), bias_noise_);
    graph_.add(factor);
    graphValues.insert(B(curr_index_ + 1), gtsam::imuBias::ConstantBias());

    // Add imu factor
    gtsam::ImuFactor imufac(X(curr_index_), V(curr_index_), X(curr_index_ + 1), V(curr_index_ + 1), B(curr_index_ + 1),
                            accum_);

    graph_.add(imufac);
    auto newPoseEstimate = result_.at<gtsam::Pose3>(X(curr_index_));  // gtsam::Pose3 newPoseEstimate
    newPoseEstimate =
        gtsam::Pose3(accum_.deltaRij() * newPoseEstimate.rotation(), accum_.deltaPij() + newPoseEstimate.translation());
    graphValues.insert(X(curr_index_ + 1), newPoseEstimate);

    Vec3 last_vel = result_.at<Vec3>(V(curr_index_));
    last_vel += accum_.deltaVij();
    graphValues.insert(V(curr_index_ + 1), last_vel);

    curr_index_++;
    optimize();
  }
  accum_.resetIntegration();
}

/**
 * Triggers ISAM2 to optimize the current graph and publish the current estimated pose
 */
void Slam::optimize()
{
#if defined(_DEBUG)
  static int iteration = 0;
  ROS_INFO_STREAM("SLAM: Iteration:" << iteration++ << " Imu_updated: " << imu_update_available_
                                     << " curr_index: " << curr_index_);
#endif
  // Adds new factors, updating the ISAM solution and relinearizing as needed.
  isam_.update(graph_, graphValues);

#if defined(_DEBUG)
  // Add initial estimates to history_
  history_.insert(graphValues);
  graph_.printErrors(history_);

  ROS_INFO_STREAM("printErrors:");
  isam_.getFactorsUnsafe().printErrors(history_);

  ROS_INFO_STREAM("isam_ graph");
  isam_.getFactorsUnsafe().print();
#endif

  result_ = isam_.calculateEstimate();

#if defined(_DEBUG)
  graph_.printErrors(graphValues);

  // Update variables with optimized ones
  history_.update(result_);
#endif

  // Clear the objects holding new factors and node values for the next iteration
  graph_.resize(0);
  graphValues.clear();

  // Discard first frames, while we wait for pose to converge
  if (curr_index_ < n_discard_frames)
  {
    ROS_INFO("Discarding initial poses...");
    return;
  }

  auto curr_pose = result_.at<gtsam::Pose3>(X(curr_index_));  // gtsam::Pose3 currPose
  Vec3 curr_vel = result_.at<Vec3>(V(curr_index_));

  // converts linear velocity to the local frame
  auto global_to_local = curr_pose.rotation().toQuaternion().inverse();
  curr_vel = global_to_local._transformVector(curr_vel);

  Vec3 curr_ang = accum_.deltaRij().rpy() / accum_.deltaTij();
  auto odom_message = createOdomMsg(curr_pose, curr_vel, curr_ang);
#if defined(_DEBUG)
  ROS_INFO_STREAM("IMU MSG: x: " << odom_message.pose.pose.position.x);
#endif

  location_pub_.publish(odom_message);
  updateTransform(odom_message);
}

void Slam::updateTransform(const nav_msgs::Odometry &pos)
{
  geometry_msgs::TransformStamped tfLink;
  tfLink.header.stamp = pos.header.stamp;
  tfLink.header.frame_id = pos.header.frame_id;
  tfLink.child_frame_id = pos.child_frame_id;

  tfLink.transform.translation.x = pos.pose.pose.position.x;
  tfLink.transform.translation.y = pos.pose.pose.position.y;
  tfLink.transform.translation.z = pos.pose.pose.position.z;
  tfLink.transform.rotation = pos.pose.pose.orientation;
  world_transform_broadcaster_.sendTransform(tfLink);
}

/**
 * Sets the IMU Params
 */
void Slam::initializeImuParams()
{
  // Should be replaced with actual imu measurements. Values should come from the launch file.
  auto params = gtsam::PreintegrationParams::MakeSharedU(KGRAVITY);
  params->setAccelerometerCovariance(gtsam::I_3x3 * 0.1);
  params->setGyroscopeCovariance(gtsam::I_3x3 * 0.1);
  params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(Vec3(0, 0, 0));
  accum_ = gtsam::PreintegratedImuMeasurements(params);
}

/**
 * Adds initial values of variables in the factor graph.
 */
void Slam::initializePriors()
{
  // Adding Initial Position (Pose + Covariance Matrix)
  sensor_msgs::Imu initImuMsg;
  sensor_msgs::ImuConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Imu>("/magnetometer", ros::Duration(1));
  if (msg == NULL)
  {
    ROS_INFO("No messages received");
  }
  else
  {
    initImuMsg = *msg;
  }
  initOrientation = gtsam::Rot3::Quaternion(initImuMsg.orientation.w, initImuMsg.orientation.x,
                                            initImuMsg.orientation.y, initImuMsg.orientation.z);
  gtsam::Pose3 priorPose = gtsam::Pose3(initOrientation, gtsam::Point3());
  noiseDiagonal::shared_ptr poseNoise =
      noiseDiagonal::Sigmas((gtsam::Vector(6) << Vec3::Constant(0.1), Vec3::Constant(0.3)).finished());
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(X(0), priorPose, poseNoise));
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<gtsam::Pose3> on x0");
#endif
  graphValues.insert(X(0), priorPose);

  // Adding Initial Velocity (Pose + Covariance Matrix)
  Vec3 priorVel(0.0, 0.0, 0.0);
  noiseDiagonal::shared_ptr velNoise = noiseDiagonal::Sigmas(Vec3::Constant(0.1));
  graph_.push_back(gtsam::PriorFactor<Vec3>(V(0), priorVel, velNoise));
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<gtsam::Vec3> on v0");
#endif
  graphValues.insert(V(0), priorVel);

  // Adding Bias Prior
  noiseDiagonal::shared_ptr biasNoise = noiseDiagonal::Sigmas(gtsam::Vector6::Constant(0.1));
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasprior(B(0), gtsam::imuBias::ConstantBias(), biasNoise);
  graph_.push_back(biasprior);
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<constantBias> on B0");
#endif
  graphValues.insert(B(0), gtsam::imuBias::ConstantBias());

  // Set flag for first reading
  firstReading = true;

  optimize();
  ROS_INFO_STREAM("Priors Initialized.");
}

/**
 * Initializes the shared noise matrices from parameters in the launch file
 */
void Slam::initializeNoiseMatrices()
{
  double bias_noise = pnh_.param("biasNoiseConst", 0.03);
  double gps_xy_noise = pnh_.param("gpsXYNoiseConstant", 0.15);
  double gps_z_noise = pnh_.param("gpsZNoiseConstant", 0.15);
  double mag_noise = pnh_.param("magNoiseConstant", 0.00000005);
  gps_noise_ = noiseDiagonal::Sigmas(Vec3(gps_xy_noise, gps_xy_noise, gps_z_noise));
  mag_noise_ = noiseDiagonal::Sigmas(Vec3::Constant(mag_noise));
  bias_noise_ = noiseDiagonal::Sigmas(gtsam::Vector6::Constant(bias_noise));
}

/**
 * Initialize the local magnetic field direction from launch params
 */
void Slam::initializeDirectionOfLocalMagField()
{
  std::vector<double> lmg =
      pnh_.param("localMagneticField", std::vector<double>{ 0.0000227095, 0.0000020783, -0.0000432753 });
  local_mag_field_ = gtsam::Unit3(lmg[0], lmg[1], lmg[2]);
  scale_ = Vec3(lmg[0], lmg[1], lmg[2]).norm();
  ROS_INFO_STREAM("Initializing Direction of Local Magnetic Field");
}

nav_msgs::Odometry Slam::createOdomMsg(const gtsam::Pose3 &pos, const gtsam::Vector3 &vel, const gtsam::Vector3 &ang)
{
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.child_frame_id = "base_footprint";
  msg.header.frame_id = "odom";
  msg.pose.pose = Conversion::gtsamPose3ToPose3Msg(pos);
  msg.twist.twist.linear = Conversion::gtsamVector3ToVector3Msg(vel);
  msg.twist.twist.angular = Conversion::gtsamVector3ToVector3Msg(ang);
  return msg;
}