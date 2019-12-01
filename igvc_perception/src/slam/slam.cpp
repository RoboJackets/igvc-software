#include <slam/slam.h>

// Using statements
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;

Slam::Slam() : pnh_{ "~" } {
    odom_sub_ = pnh_.subscribe("/wheel_odometry", 1, &Slam::OdomCallback, this);
    imu_sub_ = pnh_.subscribe("/imu", 1, &Slam::ImuCallback, this);
    location_pub = pnh_.advertise<nav_msgs::Odometry>("/slam/position", 1);

    pose_index_ = 0;
    imu_bias_counter_ = 0;
    imu_recieved_ = false;
    BIAS_UPDATE_RATE = pnh_.param("biasUpdateRatio", 20);
    BIAS_NOISE_CONST = pnh_.param("biasNoiseConst", 0.001);

    InitializeImuParams();
    InitializePriors();
}

void Slam::ImuCallback(const sensor_msgs::Imu &msg){
    ROS_INFO_STREAM("Imu called!");
    ros::Time currTime = ros::Time::now();
    gtsam::Vector3 measuredAcc = gtsam::Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z);
    gtsam::Vector3 measuredOmega = gtsam::Vector3(msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z);
    if (!imu_recieved_){
        imu_recieved_ = true;
        accum.integrateMeasurement(measuredAcc, measuredOmega, 0.005);
    } else {
        double deltaT = (currTime-lastImuMeasurement).toSec();
        accum.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    }
    lastImuMeasurement = currTime;
}

void Slam::OdomCallback(const nav_msgs::Odometry &msg){
    if (imu_recieved_) {
        // Handle the odometry
        gtsam::Pose2 currPose = gtsam::Pose2(msg.pose.pose.position.x, msg.pose.pose.position.y,
                tf::getYaw(msg.pose.pose.orientation));
        gtsam::Pose2 odometry = previousPose.between(currPose);
        noiseDiagonal::shared_ptr odometryNoise = noiseDiagonal::Variances(gtsam::Vector3(
                0.02, 0.02, 0.01));
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(pose_index_, pose_index_ + 1, odometry, odometryNoise);
        auto newPoseEstimate = currEstimate.at<gtsam::Pose2>(pose_index_); //gtsam::Pose2 newPoseEstimate
        newPoseEstimate = odometry * newPoseEstimate;
        currEstimate.insert(pose_index_+1, newPoseEstimate);

        // Add bias factor
        auto cov = noiseDiagonal::Variances(gtsam::Vector6::Constant(BIAS_NOISE_CONST));
        auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(B(pose_index_),
                B(pose_index_+1), gtsam::imuBias::ConstantBias(), cov);
        graph.add(factor);
        currEstimate.insert(B(pose_index_+1), gtsam::imuBias::ConstantBias());

        // Add imu factor
        gtsam::ImuFactor imufac(X(pose_index_), V(pose_index_), X(pose_index_+1),
                V(pose_index_+1), B(pose_index_+1), accum);
        graph.add(imufac);
        auto lastVel = currEstimate.at<gtsam::Vector2>(V(pose_index_));
        lastVel.x() += accum.deltaVij().x();
        lastVel.y() += accum.deltaVij().y();
        currEstimate.insert(V(pose_index_+1), lastVel);
        accum.resetIntegration();

        pose_index_++;
        previousPose = currPose;
        Optimize();
    }
}

void Slam::Optimize() {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, currEstimate);
    currEstimate = optimizer.optimize();
    auto currPose = currEstimate.at<gtsam::Pose2>(pose_index_); //gtsam::Pose2 currPose
    nav_msgs::Odometry msg;
    msg.child_frame_id = "/base_footprint";
    msg.header.frame_id = "/odom";
    msg.pose.pose.position.x = currPose.x();
    msg.pose.pose.position.y = currPose.y();
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(currPose.theta());
    location_pub.publish(msg);
}

void Slam::InitializeImuParams() {
    // Should be replaced with actual imu measurements
    boost::shared_ptr<gtsam::PreintegrationParams> params;
    params = gtsam::PreintegrationParams::MakeSharedU(KGRAVITY);
    params->setAccelerometerCovariance(gtsam::I_3x3 * 0.1);
    params->setGyroscopeCovariance(gtsam::I_3x3 * 0.1);
    params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);
    params->setUse2ndOrderCoriolis(false);
    params->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
    accum = gtsam::PreintegratedImuMeasurements(params);
}

void Slam::InitializePriors(){
    // Adding Initial Position (Pose + Covariance Matrix)
    gtsam::Pose2 priorPose(0.0, 0.0, 0.0);
    noiseDiagonal::shared_ptr poseNoise = noiseDiagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
    graph.push_back(gtsam::PriorFactor<gtsam::Pose2>(X(0), priorPose, poseNoise));
    currEstimate.insert(X(0), priorPose);

    // Adding Initial Velocity (Pose + Covariance Matrix)
    gtsam::Vector2 priorVel(0.0, 0.0);
    noiseDiagonal::shared_ptr velNoise = noiseDiagonal::Sigmas(gtsam::Vector2(0.1, 0.1));
    graph.push_back(gtsam::PriorFactor<gtsam::Vector2>(V(0), priorVel, velNoise));
    currEstimate.insert(V(0), priorVel);

    // Adding Bias Prior
    noiseDiagonal::shared_ptr biasNoise = noiseDiagonal::Sigmas(gtsam::Vector6::Constant(0.1));
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasprior(B(0), gtsam::imuBias::ConstantBias(),
            biasNoise);
    graph.push_back(biasprior);
    currEstimate.insert(B(0), gtsam::imuBias::ConstantBias());

    ROS_INFO_STREAM("Values initialized!");
}