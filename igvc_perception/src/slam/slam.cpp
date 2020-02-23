#include <slam/slam.h>

// Using statements
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;

//Initializes the factor graph and the node
Slam::Slam() : pnh_{ "~" } {
    odom_sub_ = pnh_.subscribe("/wheel_odometry", 1, &Slam::OdomCallback, this);
    imu_sub_ = pnh_.subscribe("/imu", 1, &Slam::ImuCallback, this);
    location_pub = pnh_.advertise<nav_msgs::Odometry>("/slam/position", 1);

    curr_index_ = 0;
    last_imu_index_ = 0;
    imu_connected_ = false;
    imu_update_available_ = false;
    BIAS_NOISE_CONST = pnh_.param("biasNoiseConst", 0.001);

    InitializeImuParams();
    InitializePriors();
}

//The ImuCallback adds IMU measurements to the accumulator when it recieves an IMU measurement
void Slam::ImuCallback(const sensor_msgs::Imu &msg){
    //ROS_INFO_STREAM("Imu called!");
    ros::Time currTime = ros::Time::now();
    Vec3 measuredAcc = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Vec3 measuredOmega = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    if (!imu_connected_){
        imu_connected_ = true;
        accum.integrateMeasurement(measuredAcc, measuredOmega, 0.005);
    } else {
        double deltaT = (currTime-lastImuMeasurement).toSec();
        if (deltaT > 0)
            accum.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
    }
    imu_update_available_ = true;
    lastImuMeasurement = currTime;
}

//The OdomCallback adds an Odometry measurement and adds an integrated IMU factor to the factor graph
void Slam::OdomCallback(const nav_msgs::Odometry &msg){
    if (imu_connected_) {
        // Handle the odometry
        gtsam::Pose3 currPose = Conversion::getPose3FromOdom(msg);
        gtsam::Pose3 odometry = previousPose.between(currPose);
        noiseDiagonal::shared_ptr odometryNoise = noiseDiagonal::Sigmas((gtsam::Vector(6)
                << Vec3::Constant(0.1), Vec3::Constant(0.3)).finished());
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(curr_index_), X(curr_index_ + 1), odometry, odometryNoise);
        auto newPoseEstimate = result.at<gtsam::Pose3>(X(curr_index_)); //gtsam::Pose3 newPoseEstimate
        newPoseEstimate = odometry * newPoseEstimate;
        initEstimate.insert(X(curr_index_ + 1), newPoseEstimate);

        if (imu_update_available_) {
            // Add bias factor
            auto cov = noiseDiagonal::Variances(gtsam::Vector6::Constant(BIAS_NOISE_CONST));
            auto factor = boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> >(B(last_imu_index_),
                                                                                                  B(curr_index_ + 1),
                                                                                                  gtsam::imuBias::ConstantBias(),
                                                                                                  cov);
            graph.add(factor);
            initEstimate.insert(B(curr_index_ + 1), gtsam::imuBias::ConstantBias());

            // Add imu factor
            gtsam::ImuFactor imufac(X(last_imu_index_), V(last_imu_index_), X(curr_index_ + 1),
                                    V(curr_index_ + 1), B(curr_index_ + 1), accum);
            graph.add(imufac);
            Vec3 lastVel = result.at<gtsam::Vector3>(V(last_imu_index_));
            lastVel += accum.deltaVij();
            last_imu_index_ = curr_index_ + 1;
            initEstimate.insert(V(last_imu_index_), lastVel);
            accum.resetIntegration();
        }

        curr_index_++;
        previousPose = currPose;
        Optimize();
        imu_update_available_ = false;
    }
}

//Triggers ISAM2 to optimize the current graph and publish the current estimated pose
void Slam::Optimize() {
    static int iteration = 0;
    if(last_imu_index_ == curr_index_){
        ROS_INFO_STREAM("SLAM: Iteration:" << iteration++ << " Imu_updated: " << imu_update_available_
        << " curr_index: " << curr_index_ << " last_imu_index: " << last_imu_index_);
    } else {
        ROS_WARN_STREAM("SLAM: Iteration:" << iteration++ << " Imu_updated: " << imu_update_available_
        << " curr_index: " << curr_index_ << " last_imu_index: " << last_imu_index_);
    }
//    graph.print();
    isam.update(graph, initEstimate);
    result = isam.calculateEstimate();
    graph.resize(0);
    initEstimate.clear();

    auto currPose = result.at<gtsam::Pose3>(X(curr_index_)); //gtsam::Pose2 currPose
    location_pub.publish(Conversion::getOdomFromPose3(currPose));
}

//Sets the IMU Params
void Slam::InitializeImuParams() {
    // Should be replaced with actual imu measurements. Values should come from the launch file.
    auto params = gtsam::PreintegrationParams::MakeSharedU(KGRAVITY);
    params->setAccelerometerCovariance(gtsam::I_3x3 * 0.1);
    params->setGyroscopeCovariance(gtsam::I_3x3 * 0.1);
    params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);
    params->setUse2ndOrderCoriolis(false);
    params->setOmegaCoriolis(Vec3(0, 0, 0));
    accum = gtsam::PreintegratedImuMeasurements(params);
}

//Adds initial values of variables in the factor graph.
void Slam::InitializePriors(){
    // Adding Initial Position (Pose + Covariance Matrix)
    gtsam::Pose3 priorPose;
    noiseDiagonal::shared_ptr poseNoise = noiseDiagonal::Sigmas((gtsam::Vector(6)
            << Vec3::Constant(0.1), Vec3::Constant(0.3)).finished());
    graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(X(0), priorPose, poseNoise));
    initEstimate.insert(X(0), priorPose);

    // Adding Initial Velocity (Pose + Covariance Matrix)
    Vec3 priorVel(0.0, 0.0, 0.0);
    noiseDiagonal::shared_ptr velNoise = noiseDiagonal::Sigmas(Vec3::Constant(0.1));
    graph.push_back(gtsam::PriorFactor<Vec3>(V(0), priorVel, velNoise));
    initEstimate.insert(V(0), priorVel);

    // Adding Bias Prior
    noiseDiagonal::shared_ptr biasNoise = noiseDiagonal::Sigmas(gtsam::Vector6::Constant(0.1));
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasprior(B(0), gtsam::imuBias::ConstantBias(),
            biasNoise);
    graph.push_back(biasprior);
    initEstimate.insert(B(0), gtsam::imuBias::ConstantBias());

    Optimize();
    ROS_INFO_STREAM("Priors Initialized.");
}