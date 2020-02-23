#ifndef SRC_SLAM_H
#define SRC_SLAM_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// Pose2 == (Point3, Rot3)
#include <gtsam/geometry/Pose3.h>
// PriorFactor == Initial Pose
#include <gtsam/slam/PriorFactor.h>
// BetweenFactor == Odom measurement
#include <gtsam/slam/BetweenFactor.h>
// The factor graph we are creating. Nonlinear since angle measurements are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
// Helps initialize initial guess
#include <gtsam/nonlinear/Values.h>
#include <slam/slam.h>
#include <tf/transform_listener.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <slam/type_conversions.h>

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
    typedef gtsam::Vector3 Vec3;

    // Establishing global variables
    gtsam::Values initEstimate, result;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Pose3 previousPose;
    unsigned long curr_index_;
    unsigned long last_imu_index_;
    double BIAS_NOISE_CONST;
    gtsam::ISAM2 isam;
    const double KGRAVITY = 9.81;
    gtsam::PreintegratedImuMeasurements accum;
    ros::Time lastImuMeasurement;
    bool imu_connected_;
    bool imu_update_available_;
};

#endif //SRC_SLAM_H
