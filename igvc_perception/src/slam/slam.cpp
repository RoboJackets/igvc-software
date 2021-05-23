#include <slam/slam.h>

// Using statements
using gtsam::symbol_shorthand::B; // bias measurements
using gtsam::symbol_shorthand::V; // velocity measurements
using gtsam::symbol_shorthand::X; // pose measurements

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
  gps_location_pub_ = pnh_.advertise<nav_msgs::Odometry>("/slam_gps", 1);

//  tf2_filter_ = tf2_ros::MessageFilter<sensor_msgs::NavSatFix>(buffer_, target_frame_, 10, pnh_);
  curr_index_ = 0;
  imu_connected_ = false;
  imu_update_available_ = false;
  transformToOdom = getTransform(target_frame_, ros::Time::now());

  initializeDirectionOfLocalMagField();
  initializeNoiseMatrices();
  initializeImuParams();
  initializePriors();
}

/**
 * The gpsCallback adds GPS measurements to the factor graph
 * @param gps A NavSatFix message derived from GPS measurements (published by /fix)
 */
void Slam::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps) {
    // get the starting location as the origin
    double lat, lon, h, E, N, U;
    lat = gps->latitude;
    lon = gps->longitude;
    h = gps->altitude;
    ROS_INFO_STREAM("gps measurement number: " << gps->header.seq);
    if (firstReading) {
        const double lat0 = 33.86998, lon0 = -84.30626, h0 = 274;
        GeographicLib::LocalCartesian test_ENU = GeographicLib::LocalCartesian(lat0, lon0, h0, GeographicLib::Geocentric::WGS84());
        test_ENU.Forward(33.87071, -84.30482, 274, E, N, U);
        ROS_INFO_STREAM("TEST gps_factor reading: " << E << ", " << N << ", " << U);

        origin_ENU = GeographicLib::LocalCartesian(lat, lon, h, GeographicLib::Geocentric::WGS84());
        firstReading = false;
//        origin_ENU = GeographicLib::LocalCartesian(gps->latitude, gps->longitude, gps->altitude, GeographicLib::Geocentric::WGS84());
    }
    origin_ENU.Forward(lat, lon, h, E, N, U);

    geometry_msgs::Quaternion quat = transformToOdom.transform.rotation;
    gtsam::Rot3 global_to_local = gtsam::Rot3::Quaternion(quat.x, quat.y, quat.z, quat.w).inverse();

//    gtsam::Rot3 global_to_local = gtsam::Rot3::Quaternion(0,0,-.707,.707);

//    Vec3 measured_gps = Vec3(E, N, U);
//    Vec3 curr_gps = global_to_local * measured_gps;
    gtsam::Point3 measured_gps = gtsam::Point3(E, N, U);
    gtsam::Point3 curr_gps = global_to_local.rotate(measured_gps);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
//    odom.pose.pose.position.x = curr_gps.x();
//    odom.pose.pose.position.y = curr_gps.y();
//    odom.pose.pose.position.z = curr_gps.z();
    odom.pose.pose.position.x = E;
    odom.pose.pose.position.y = N;
    odom.pose.pose.position.z = U;
    gps_location_pub_.publish(odom);

//    origin_ENU.Forward(gps->latitude, gps->longitude, gps->altitude, E, N, U);
//    gtsam::Point3 curr_point = Conversion::odomMsgToGtsamPoint3(msg);
//  gtsam::GPSFactor gps_factor(X(curr_index_ + 1), curr_point, gps_noise_);
//    ROS_INFO_STREAM("gps_latlon reading: " << lat << ", " << lon << ", " << h);
//    ROS_INFO_STREAM("gps_pointer reading: " << gps->latitude << ", " << gps->longitude << ", " << gps->altitude);
    ROS_INFO_STREAM("gps_factor reading: " << E << ", " << N << ", " << U);
//    gtsam::GPSFactor gps_factor(X(curr_index_ + 1), gtsam::Point3(curr_gps.x(), curr_gps.y(), curr_gps.z()), gps_noise_);
    gtsam::GPSFactor gps_factor(X(curr_index_ + 1), gtsam::Point3(E, N, U), gps_noise_);
  graph_.add(gps_factor);

  addMagFactor();

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
//    gtsam::Rot3 quat = gtsam::Rot3::Quaternion(0,0,.707,.707);
//    gtsam::Point3 curr = gtsam::Point3(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
//    curr_mag_reading_ = quat.rotate(curr);
  curr_mag_reading_ = gtsam::Point3(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
}

/**
 * Adds the magFactor to the factor graph
 */
void Slam::addMagFactor()
{
  MagPoseFactor mag_factor(X(curr_index_ + 1), curr_mag_reading_, scale_,
      local_mag_field_, gtsam::Point3(1e-9, 1e-9, 1e-9),mag_noise_);
  graph_.add(mag_factor);
}

/**
 * If there are IMU measurements in the accumulator, this adds them as a single factor to the factor graph.
 */
void Slam::integrateAndAddIMUFactor()
{
  if (accum_.preintMeasCov().trace() != 0)
  {
    // Add bias factor
    auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(
        B(curr_index_), B(curr_index_ + 1), gtsam::imuBias::ConstantBias(), bias_noise_);
    graph_.add(factor);
    init_estimate_.insert(B(curr_index_ + 1), gtsam::imuBias::ConstantBias());

    // Add imu factor
    gtsam::ImuFactor imufac(X(curr_index_), V(curr_index_), X(curr_index_ + 1),
        V(curr_index_ + 1), B(curr_index_ + 1),accum_);

    graph_.add(imufac);
    auto newPoseEstimate = result_.at<gtsam::Pose3>(X(curr_index_));  // gtsam::Pose3 newPoseEstimate
    newPoseEstimate =
        gtsam::Pose3(accum_.deltaRij() * newPoseEstimate.rotation(), accum_.deltaPij() + newPoseEstimate.translation());
    init_estimate_.insert(X(curr_index_ + 1), newPoseEstimate);
    Vec3 last_vel = result_.at<Vec3>(V(curr_index_));
    last_vel += accum_.deltaVij();
    init_estimate_.insert(V(curr_index_ + 1), last_vel);
    curr_index_++;
    optimize();
  }
  accum_.resetIntegration();
}

/**
 * The magCallback updates the curr_mag_reading_ with its most recent value
 * @param msg An MagneticField sensor message (published by yostlab_driver_node)
 */
void Slam::wheelOdomCallback(const nav_msgs::Odometry &msg)
{
    curr_wheelOdom_reading_ = Conversion::odomMsgToGtsamPose2(msg);
    ROS_INFO_STREAM("Wheel odom reading: " << curr_wheelOdom_reading_.x() << ", " << curr_wheelOdom_reading_.y() << ", " << curr_wheelOdom_reading_.theta());
}

/**
 * Adds the magFactor to the factor graph
 */
void Slam::addWheelOdomFactor()
{
//    gtsam::BetweenFactor<gtsam::Pose2> odomFactor = gtsam::BetweenFactor<gtsam::Pose2>(X(curr_index_), X(curr_index_ + 1), curr_wheelOdom_reading_, odometryNoise);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(X(curr_index_), X(curr_index_ + 1), curr_wheelOdom_reading_, odometryNoise));
//    gtsam::OdometryFactorBase odomFactor = gtsam::OdometryFactorBase()
//    MagPoseFactor mag_factor(X(curr_index_ + 1), curr_mag_reading_, scale_,
//                             local_mag_field_, gtsam::Point3(1e-9, 1e-9, 1e-9),mag_noise_);
//    graph_.add(mag_factor);
}

/**
 * Triggers ISAM2 to optimize the current graph and publish the current estimated pose
 */
void Slam::optimize()
{
  static int iteration = 0;
  ROS_INFO_STREAM("SLAM: Iteration:" << iteration++ << " Imu_updated: " << imu_update_available_
                                     << " curr_index: " << curr_index_);
  // Update ISAM graph with new factors and estimates
  isam_.update(graph_, init_estimate_);

  #if defined(_DEBUG)
  // Add initial estimates to history_
  history_.insert(init_estimate_);
  graph_.printErrors(history_);

  ROS_INFO_STREAM("printErrors:");
  isam_.getFactorsUnsafe().printErrors(history_);

  ROS_INFO_STREAM("isam_ graph");
  isam_.getFactorsUnsafe().print();
  #endif

  result_ = isam_.calculateEstimate();

#if defined(_DEBUG)
  graph_.printErrors(init_estimate_);

  // Update variables with optimized ones
  history_.update(result_);
#endif

  // Clear graph and estimates
  graph_.resize(0);
  init_estimate_.clear();

  auto curr_pose = result_.at<gtsam::Pose3>(X(curr_index_));  // gtsam::Pose3 currPose
  Vec3 curr_vel = result_.at<Vec3>(V(curr_index_));

  // converts linear velocity to the local frame
  auto global_to_local = curr_pose.rotation().toQuaternion().inverse();
  curr_vel = global_to_local._transformVector(curr_vel);

  Vec3 curr_ang = accum_.deltaRij().rpy()/accum_.deltaTij();
  auto odom_message = createOdomMsg(curr_pose, curr_vel, curr_ang);
//  ROS_INFO_STREAM("IMU MSG: x: " << odom_message.pose.pose.position.x);
  location_pub_.publish(odom_message);
  updateTransform(odom_message);
}

void Slam::updateTransform(const nav_msgs::Odometry &pos){
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
  gtsam::Pose3 priorPose;
  noiseDiagonal::shared_ptr poseNoise =
      noiseDiagonal::Sigmas((gtsam::Vector(6) << Vec3::Constant(0.1), Vec3::Constant(0.3)).finished());
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(X(0), priorPose, poseNoise));
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<gtsam::Pose3> on x0");
#endif
  init_estimate_.insert(X(0), priorPose);
  firstReading = true;

  // Adding Initial Velocity (Pose + Covariance Matrix)
  Vec3 priorVel(0.0, 0.0, 0.0);
  noiseDiagonal::shared_ptr velNoise = noiseDiagonal::Sigmas(Vec3::Constant(0.1));
  graph_.push_back(gtsam::PriorFactor<Vec3>(V(0), priorVel, velNoise));
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<gtsam::Vec3> on v0");
#endif
  init_estimate_.insert(V(0), priorVel);

  // Adding Bias Prior
  noiseDiagonal::shared_ptr biasNoise = noiseDiagonal::Sigmas(gtsam::Vector6::Constant(0.1));
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasprior(B(0), gtsam::imuBias::ConstantBias(), biasNoise);
  graph_.push_back(biasprior);
#if defined(_DEBUG)
  ROS_INFO_STREAM("Factor " << graph_.size() << ": PriorFactor<constantBias> on B0");
#endif
  init_estimate_.insert(B(0), gtsam::imuBias::ConstantBias());

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
//  gps_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 0.25);
  mag_noise_ = noiseDiagonal::Sigmas(Vec3::Constant(mag_noise));
  odometryNoise = noiseDiagonal::Sigmas(Vec3(0.2, 0.2, 0.1));
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

geometry_msgs::TransformStamped Slam::getTransform(const std::string &frame, const ros::Time &stamp) const
{
//    geometry_msgs::TransformStamped transformStamped;
//    try{
//        transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
//                                                    ros::Time(0));
//    }
//    catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//        continue;
//    }

    if (!buffer_.canTransform("odom", frame, stamp, ros::Duration{ 1 }))
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "Failed to find transform from frame 'odom' to frame 'base_link' within "
                                      "timeout. Using latest transform...");
        return buffer_.lookupTransform("odom", frame, ros::Time{ 0 }, ros::Duration{ 1 });
    }

    return buffer_.lookupTransform("odom", frame, stamp, ros::Duration{ 1 });
}