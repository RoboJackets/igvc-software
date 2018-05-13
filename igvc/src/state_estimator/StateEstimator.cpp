#include "StateEstimator.h"
#include <thread>

using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X; // position
using symbol_shorthand::V; // velocity
using symbol_shorthand::B; // bias
using symbol_shorthand::G; // GPS position


StateEstimator::StateEstimator() :
  gpsQ_(),
  imuQ_(),
  lastIMUT_(),
  lastIMUTcb_(),
  nh_("~")
{

  imuToGps_ = Pose3(Rot3::identity(), Point3(-.044, .214, 0));

  posePub_ = nh_.advertise<nav_msgs::Odometry>("pose", 1);

  imuSub_ = nh_.subscribe("imu", 600, &StateEstimator::imuCallback, this);
  gpsSub_ = nh_.subscribe("gps", 300, &StateEstimator::gpsCallback, this);

  std::thread(&StateEstimator::optimizationLoop, this);
}

void StateEstimator::optimizationLoop()
{

  ISAM2Params parameters;
  //parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
  //parameters.relinearizeSkip = 1; // Relinearize every time
  gtsam::IncrementalFixedLagSmoother graph(20, parameters);

  double startTime;
  sensor_msgs::ImuConstPtr lastImu;
  double lastImuT;
  int imuKey = 1;
  

  // first we will initialize the graph with appropriate priors
  NonlinearFactorGraph priorFactors;
  Values priorVariables;

  sensor_msgs::NavSatFixConstPtr fix = gpsQ_.pop();
  startTime = ROS_TIME(fix);
  enu_.Reset(fix->latitude, fix->longitude, fix->altitude);

  sensor_msgs::ImuConstPtr imu = imuQ_.pop();
  lastImu = imu;
  lastImuT = ROS_TIME(imu);
  Rot3 initialOrientation = Rot3::Quaternion(
      imu->orientation.w,
      imu->orientation.x,
      imu->orientation.y,
      imu->orientation.z);

  // we set out initial position to the origin and assume we are stationary
  Pose3 x0(initialOrientation, Point3(0,0,0));
  PriorFactor<Pose3> priorPose(X(0), x0,
      noiseModel::Diagonal::Sigmas( (Vector(6) << 0.25,0,25,0.25, 0.01, 0.01, 0.01).finished() ));
  priorFactors.add(priorPose);

  Vector3 v0 = Vector3(0,0,0);
  PriorFactor<Vector3> priorVel(V(0), v0,
      noiseModel::Diagonal::Sigmas( (Vector(3) << 0.01, 0.01, 0.01).finished() ));
  priorFactors.add(priorVel);

  imuBias::ConstantBias b0( (Vector(6) << 0, 0, 0, 0, 0, 0).finished() );
  PriorFactor<imuBias::ConstantBias> priorBias(B(0), b0,
      noiseModel::Diagonal::Sigmas( (Vector(6) << 0.4, 0.4, 0.4, 0.2, 0.2, 0.2).finished() ));
  priorFactors.add(priorBias);

  BetweenFactor<Pose3> imuToGpsFactor(X(0), G(0), imuToGps_,
      noiseModel::Diagonal::Sigmas( (Vector(6) << 1e-2, 1e-2, 1e-2, 3e-2, 3e-2, 3e-2).finished() ));
  priorFactors.add(imuToGpsFactor);

  priorVariables.insert(X(0), x0);
  priorVariables.insert(V(0), v0);
  priorVariables.insert(B(0), b0);
  priorVariables.insert(G(0), x0.compose(imuToGps_));

  graph.update(priorFactors, priorVariables);

  Pose3 prevPose = x0;
  Vector3 prevVel = v0;
  imuBias::ConstantBias prevBias = b0;

  // remove old imu messages
  while (!imuQ_.empty() && ROS_TIME(imuQ_.front()) < ROS_TIME(fix))
    lastImu = imuQ_.pop();

  // setting up the IMU integration
  boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams =  PreintegrationParams::MakeSharedU(9.80511); // magnitude of gravity looked up from wolframalpha
  preintegrationParams->accelerometerCovariance = 1e-2 * I_3x3;
  preintegrationParams->gyroscopeCovariance = 1e-3 * I_3x3;
  preintegrationParams->integrationCovariance = 1e-4 * I_3x3;

  boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator(preintegrationParams, prevBias);

  // now we loop and let use the queues to grab messages
  while (ros::ok())
  {
    NonlinearFactorGraph newFactors;
    Values newVariables;


    // integrate imu messages
    while (!imuQ_.empty() && ROS_TIME(imuQ_.back()) > (startTime + 0.1*imuKey))
    {
      imu = imuQ_.pop();
      dt = ROS_TIME(imu) - lastImuT;
      lastImuT = ROS_TIME(imu);
      imuIntegrator.integrateMeasurement(
          Vector3(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z),
          Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z),
          dt);
    }

  }

}

void StateEstimator::gpsCallback(sensor_msgs::NavSatFixConstPtr fix)
{
  gpsQ_.push(fix);
}

void StateEstimator::imuCallback(sensor_msgs::ImuConstPtr imu)
{
  imuQ_.push(imu);
}

StateEstimator::~StateEstimator() {}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "StateEstimator");
  StateEstimator st;
  ros::spin();
}
