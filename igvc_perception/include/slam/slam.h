#ifndef SRC_SLAM_H
#define SRC_SLAM_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// Pose2 == (x, y, theta)
#include <gtsam/geometry/Pose2.h>
// PriorFactor == Initial Pose
#include <gtsam/slam/PriorFactor.h>
// BetweenFactor == Odom measurement
#include <gtsam/slam/BetweenFactor.h>
// The factor graph we are creating. Nonlinear since angle measurements are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// May want to choose a different optimizer later, but for now we use this one.
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
// Helps initialize initial guess
#include <gtsam/nonlinear/Values.h>
#include <slam/slam.h>
#include <tf/transform_listener.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

class Slam {
public:
    Slam();

private:
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    //ros::Subscriber gps_sub_;

    ros::Publisher location_pub;


    void ImuCallback(const sensor_msgs::Imu &msg);
    void OdomCallback(const nav_msgs::Odometry &msg);
    void Optimize();
    void InitializeImuParams();
    void InitializePriors();

    // Defining some types
    typedef gtsam::noiseModel::Diagonal noiseDiagonal;

    // Establishing global variables
    gtsam::Values currEstimate;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Pose2 previousPose;
    unsigned long pose_index_;
    int imu_bias_counter_;
    int BIAS_UPDATE_RATE;
    double BIAS_NOISE_CONST;
    gtsam::ISAM2 isam;
    const double KGRAVITY = 9.81;
    gtsam::PreintegratedImuMeasurements accum;
    ros::Time lastImuMeasurement;
    bool imu_recieved_;
};

#endif //SRC_SLAM_H
