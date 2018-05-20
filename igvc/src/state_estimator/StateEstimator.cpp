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
  lastImuT_(),
  imuMeasurements_(),
  mutex_(),
  doneFirstOpt_(false),
  nh_("~")
{

  imuToGps_ = Pose3(Rot3::identity(), Point3(-.044, .214, 0));

  posePub_ = nh_.advertise<nav_msgs::Odometry>("pose", 1);

  imuSub_ = nh_.subscribe("imu", 600, &StateEstimator::imuCallback, this);
  gpsSub_ = nh_.subscribe("gps", 300, &StateEstimator::gpsCallback, this);


  // setting up the IMU integration for IMU message thread
  boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams =  PreintegrationParams::MakeSharedU(0); // 9.80511); // magnitude of gravity looked up from wolframalpha
  preintegrationParams->accelerometerCovariance = 1e-2 * I_3x3;
  preintegrationParams->gyroscopeCovariance = 1e-3 * I_3x3;
  preintegrationParams->integrationCovariance = 1e-4 * I_3x3;

  imuBias::ConstantBias b( (Vector(6) << 0, 0, 0, 0, 0, 0).finished() );
  imuIntegrator_ = PreintegratedImuMeasurements(preintegrationParams, b);


  std::thread thread(&StateEstimator::optimizationLoop, this);
  ros::spin();
}

void StateEstimator::optimizationLoop()
{

  ISAM2Params parameters;
  //parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
  //parameters.relinearizeSkip = 1; // Relinearize every time
  gtsam::IncrementalFixedLagSmoother graph(1., parameters);

  double startTime;
  sensor_msgs::ImuConstPtr lastImu;
  double lastImuT;
  int imuKey = 1;
  int gpsKey = 1;
  

  // first we will initialize the graph with appropriate priors
  NonlinearFactorGraph priorFactors;
  Values priorVariables;
  FixedLagSmoother::KeyTimestampMap priorTimestamps;

  sensor_msgs::NavSatFixConstPtr fix = gpsQ_.pop();

  startTime = ROS_TIME(fix);
  enu_.Reset(fix->latitude, fix->longitude, fix->altitude);

  sensor_msgs::ImuConstPtr imu = imuQ_.pop();
  lastImu = imu;
  lastImuT = ROS_TIME(imu) - 1/125.;
  Rot3 initialOrientation = Rot3::Quaternion(
      imu->orientation.w,
      imu->orientation.x,
      imu->orientation.y,
      imu->orientation.z);

  // we set out initial position to the origin and assume we are stationary
  Pose3 x0(initialOrientation, Point3(0,0,0));
  PriorFactor<Pose3> priorPose(X(0), x0,
      noiseModel::Diagonal::Sigmas( (Vector(6) << 0.25,0.25,0.25, 0.01, 0.01, 0.01).finished() ));
  priorFactors.add(priorPose);

  Vector3 v0 = Vector3(0,0,0);
  PriorFactor<Vector3> priorVel(V(0), v0,
      noiseModel::Diagonal::Sigmas( (Vector(3) << 0.01, 0.01, 0.01).finished() ));
  priorFactors.add(priorVel);

  imuBias::ConstantBias b0( (Vector(6) << 0, 0, 0, 0, 0, 0).finished() );
  PriorFactor<imuBias::ConstantBias> priorBias(B(0), b0,
      noiseModel::Diagonal::Sigmas( (Vector(6) << 0.4, 0.4, 0.4, 0.2, 0.2, 0.2).finished() ));
  priorFactors.add(priorBias);


  noiseModel::Diagonal::shared_ptr imuToGpsFactorNoise = noiseModel::Diagonal::Sigmas( (Vector(6) << 1e-2, 1e-2, 1e-2, 3e-2, 3e-2, 3e-2).finished() );
  priorFactors.add( BetweenFactor<Pose3>(X(0), G(0), imuToGps_, imuToGpsFactorNoise) );

  priorVariables.insert(X(0), x0);
  priorVariables.insert(V(0), v0);
  priorVariables.insert(B(0), b0);
  priorVariables.insert(G(0), x0.compose(imuToGps_));

  priorTimestamps[X(0)] = 0;
  priorTimestamps[G(0)] = 0;
  priorTimestamps[V(0)] = 0;
  priorTimestamps[B(0)] = 0;

      std::cout << "X(" << 0 << "): " << priorTimestamps[X(0)] << std::endl;
      std::cout << "G(" << 0 << "): " << priorTimestamps[G(0)] << std::endl;
      std::cout << "V(" << 0 << "): " << priorTimestamps[V(0)] << std::endl;
      std::cout << "B(" << 0 << "): " << priorTimestamps[B(0)] << std::endl;

  std::cout << "adding timestamp t=0" << std::endl;

  graph.update(priorFactors, priorVariables); //, priorTimestamps);

  Pose3 prevPose = prevPose_ = x0;
  Vector3 prevVel = prevVel_ = v0;
  imuBias::ConstantBias prevBias = prevBias_ = b0;

  // remove old imu messages
  while (!imuQ_.empty() && ROS_TIME(imuQ_.front()) < ROS_TIME(fix))
  {
    lastImuT = ROS_TIME(lastImu);
    lastImu = imuQ_.pop();
  }

  // setting up the IMU integration
  boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams =  PreintegrationParams::MakeSharedU(0); // 9.80511); // magnitude of gravity looked up from wolframalpha
  preintegrationParams->accelerometerCovariance = 1e-2 * I_3x3;
  preintegrationParams->gyroscopeCovariance = 1e-3 * I_3x3;
  preintegrationParams->integrationCovariance = 1e-4 * I_3x3;

  PreintegratedImuMeasurements imuIntegrator(preintegrationParams, prevBias);

  Vector noiseModelBetweenBias = (Vector(6) << 2e-4,2e-4,2e-4, 3e-5,3e-5,3e-5).finished();

  // TODO make this a launch params
  SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(1.5, 1.5, 4.5));

  NonlinearFactorGraph newFactors;
  Values newVariables;
  FixedLagSmoother::KeyTimestampMap newTimestamps;

  // now we loop and let use the queues to grab messages
  while (ros::ok())
  {
    bool optimize = false;


    // integrate imu messages
    while (!imuQ_.empty() && ROS_TIME(imuQ_.back()) > (startTime + 0.1*imuKey) && !optimize)
    {
      double curTime = startTime + 0.1*imuKey;
      // we reset the integrator, then integrate
      imuIntegrator.resetIntegrationAndSetBias(prevBias);
      while (ROS_TIME(lastImu) < curTime)
      {
        double dt = ROS_TIME(lastImu) - lastImuT;
        imuIntegrator.integrateMeasurement(
            Vector3(lastImu->linear_acceleration.x, lastImu->linear_acceleration.y, lastImu->linear_acceleration.z),
            Vector3(lastImu->angular_velocity.x, lastImu->angular_velocity.y, lastImu->angular_velocity.z),
            dt);
        lastImuT = ROS_TIME(lastImu);
        lastImu = imuQ_.pop();
      }
      // now put this into the graph
      ImuFactor imuf( X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), imuIntegrator );
      newFactors.add(imuf);
      newFactors.add( BetweenFactor<imuBias::ConstantBias>( B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
            noiseModel::Diagonal::Sigmas( sqrt(imuIntegrator.deltaTij()) * noiseModelBetweenBias) ) );
      NavState cur(prevPose, prevVel);
      NavState next = imuIntegrator.predict(cur, prevBias);
      prevPose = next.pose();
      prevVel = next.v();
      newVariables.insert(X(imuKey), prevPose);
      newVariables.insert(G(imuKey), prevPose.compose(imuToGps_));
      newVariables.insert(V(imuKey), prevVel);
      newVariables.insert(B(imuKey), prevBias);
      // for marginalizing out past the time window
      newTimestamps[X(imuKey)] = 0.1*imuKey;
      newTimestamps[G(imuKey)] = 0.1*imuKey;
      newTimestamps[V(imuKey)] = 0.1*imuKey;
      newTimestamps[B(imuKey)] = 0.1*imuKey;
      //std::cout << "adding timestamp t=" << imuKey << std::endl;
			std::cout << "adding imu: " << imuKey << std::endl;
      ++imuKey;
      optimize = true;
    }

    while (!gpsQ_.empty() && gpsKey < imuKey && optimize && ROS_TIME(gpsQ_.back()) > (startTime + gpsKey*0.1))
    {
      fix = gpsQ_.pop();
      // we don't want all gps messages, just ones that are very close to the factors (10 hz)
      if (std::abs( ROS_TIME(fix) - (startTime+gpsKey*0.1) ) > 1e-2)
        continue;

      double E,N,U;
      enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
      // we should maybe do a check on the GPS to make sure it's valid
      newFactors.add( GPSFactor(G(gpsKey), Point3(E,N,U), gpsNoise) );
      newFactors.add( BetweenFactor<Pose3>(X(gpsKey), G(gpsKey), imuToGps_, imuToGpsFactorNoise) );
			std::cout << "adding G(" << gpsKey << ")"<< std::endl;
      ++gpsKey;
    }

    if (!optimize) continue;

    try
    {
      graph.update(newFactors, newVariables); //, newTimestamps);

      //for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: graph.timestamps()) {
        //std::cout << "key: " << key_timestamp.first << "  Time: " << key_timestamp.second << std::endl;
      //}


      prevPose = graph.calculateEstimate<Pose3>(X(imuKey-1));
      prevVel = graph.calculateEstimate<Vector3>(V(imuKey-1));
      prevBias = graph.calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

      // pass this to the other thread
      {
        std::lock_guard<std::mutex> lock(mutex_);
        prevPose_ = prevPose;
        prevVel_ = prevVel;
        prevBias_ = prevBias;
        currentTime_ = (imuKey-1) * 0.1 + startTime;
        doneFirstOpt_ = true;
      }
    }
    catch(IndeterminantLinearSystemException ex)
    {
      // optimization blew up, not much to do just warn user
      ROS_ERROR("Indeterminant linear system error");
    }

    newFactors.resize(0);
    newVariables.clear();
    newTimestamps.clear();
  }

}

void StateEstimator::gpsCallback(sensor_msgs::NavSatFixConstPtr fix)
{
  gpsQ_.push(fix);
}

void StateEstimator::imuCallback(sensor_msgs::ImuConstPtr imu)
{
  // push message onto optimization queue and integration list
  imuQ_.push(imu);
  imuMeasurements_.push_back(imu);

  // grab variables from optimization thread
  Pose3 prevPose;
  Vector3 prevVel;
  imuBias::ConstantBias prevBias;
  double currentTime;
  bool doneFirstOpt;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prevPose = prevPose_;
    prevVel = prevVel_;
    prevBias = prevBias_;
    currentTime = currentTime_;
    doneFirstOpt = doneFirstOpt_;
  }

  if (!doneFirstOpt) return;

  double dt = (lastImuT_ == 0) ? 1/125. : ROS_TIME(imu) - lastImuT_;
  lastImuT_ = ROS_TIME(imu);

  bool newState = false;
  double lastImuQT;
  while (!imuMeasurements_.empty() && ROS_TIME(imuMeasurements_.front()) < currentTime)
  {
    lastImuQT = ROS_TIME(imuMeasurements_.front());
    imuMeasurements_.pop_front();
    newState = true;
  }


  if (newState)
  {
    // we have a new optimized state to integrate from
    imuIntegrator_.resetIntegrationAndSetBias(prevBias);
    for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it)
    {
      double dt_temp = ROS_TIME(*it) - lastImuQT;
      lastImuQT = ROS_TIME(*it);
      imuIntegrator_.integrateMeasurement(
          Vector3((*it)->linear_acceleration.x, (*it)->linear_acceleration.y, (*it)->linear_acceleration.z),
          Vector3((*it)->angular_velocity.x, (*it)->angular_velocity.y, (*it)->angular_velocity.z),
          dt_temp);
    }
  }
  else
  {
    imuIntegrator_.integrateMeasurement(
        Vector3(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z),
        Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z),
        dt);
  }

  NavState prevState(prevPose, prevVel);
  NavState currentState = imuIntegrator_.predict(prevState, prevBias);

  // now we have our current state, we publish
  nav_msgs::Odometry pose;
  pose.header.stamp = imu->header.stamp;
  pose.header.frame_id = "odom";
  pose.child_frame_id = "base_link";

	Vector4 q = currentState.quaternion().coeffs();
	pose.pose.pose.orientation.x = q[0];
	pose.pose.pose.orientation.y = q[1];
	pose.pose.pose.orientation.z = q[2];
	pose.pose.pose.orientation.w = q[3];

	pose.pose.pose.position.x = currentState.position().x();
	pose.pose.pose.position.y = currentState.position().y();
	pose.pose.pose.position.z = currentState.position().z();

	pose.twist.twist.linear.x = currentState.velocity().x();
	pose.twist.twist.linear.y = currentState.velocity().y();
	pose.twist.twist.linear.z = currentState.velocity().z();
	
	pose.twist.twist.angular.x = imu->angular_velocity.x + prevBias.gyroscope().x();
	pose.twist.twist.angular.y = imu->angular_velocity.y + prevBias.gyroscope().y();
	pose.twist.twist.angular.z = imu->angular_velocity.z + prevBias.gyroscope().z();

	posePub_.publish(pose);


}

StateEstimator::~StateEstimator() {}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "StateEstimator");
  StateEstimator st;
}
