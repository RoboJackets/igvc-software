#include <slam/slam.h>


Slam::Slam() : pnh_{ "~" } {
    odom_sub_ = pnh_.subscribe("/wheel_odometry", 1, &Slam::OdomCallback, this);
    imu_sub_ = pnh_.subscribe("/imu", 1, &Slam::ImuCallback, this);
    location_pub = pnh_.advertise<nav_msgs::Odometry>("/slam/position", 1);

    pose_index_ = 0;
    odomNoiseRatio = pnh_.getParam("odomNoiseRatio", odomNoiseRatio);

    // Adding Initial Position (Pose + Covariance Matrix)
    gtsam::Pose2 priorMean(0.0, 0.0, 0.0);
    noiseDiagonal::shared_ptr priorNoise = noiseDiagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(0, priorMean, priorNoise);
    currEstimate.insert(pose_index_, priorMean);
}

void Slam::ImuCallback(const sensor_msgs::Imu &msg){

}

void Slam::OdomCallback(const nav_msgs::Odometry &msg){
    gtsam::Pose2 currPose = gtsam::Pose2(msg.pose.pose.position.x, msg.pose.pose.position.y, tf::getYaw(msg.pose.pose.orientation));
    gtsam::Pose2 odometry = previousPose.between(currPose);
    noiseDiagonal::shared_ptr odometryNoise = noiseDiagonal::Variances(gtsam::Vector3(
            0.02, 0.02, 0.01));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(pose_index_, pose_index_+1, odometry, odometryNoise);
    gtsam::Pose2 newPoseEstimate = currEstimate.at<gtsam::Pose2>(pose_index_);
    pose_index_++;
    newPoseEstimate = odometry * newPoseEstimate;
    currEstimate.insert(pose_index_, newPoseEstimate);
    previousPose = currPose;
    Optimize();
}

void Slam::Optimize() {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, currEstimate);
    currEstimate = optimizer.optimize();
    gtsam::Pose2 currPose = currEstimate.at<gtsam::Pose2>(pose_index_);
    nav_msgs::Odometry msg;
    msg.child_frame_id = "/base_footprint";
    msg.header.frame_id = "/odom";
    msg.pose.pose.position.x = currPose.x();
    msg.pose.pose.position.y = currPose.y();
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(currPose.theta());
    location_pub.publish(msg);
}