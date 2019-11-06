#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <slam/g2o/vertex/vertex_robot_state.h>
#include <slam/g2o/edge/edge_imu.h>
#include <slam/g2o/edge/edge_holonomic.h>

//Initializing global variables
g2o::SparseOptimizer optimizer_;
ros::Time starting_time_{};
int latest_pose_vertex_id_ = 0;
int latest_vertex_id_ = 0;
bool print_output = true;


/*void PointCloudCallback(const sensor_msgs::PointCloud2 &cloud){

}*/

void ImuCallback(sensor_msgs::Imu imu){

}

void OdomCallback(const geometry_msgs::PoseWithCovariance &msg)
{

}

g2o::VertexRobotState* addVertex(const g2o::RobotState& state){
    auto new_vertex = new g2o::VertexRobotState();

    int old_vertex_id = latest_pose_vertex_id_;
    int new_vertex_id = latest_vertex_id_ + 1;

    new_vertex -> setId(new_vertex_id);
    new_vertex -> setEstimate(state);

    optimizer_.addVertex(new_vertex);

    auto last_vertex = static_cast<g2o::VertexRobotState*>(optimizer_.vertex(old_vertex_id));
    auto holonomic_constraint = new g2o::EdgeHolonomic();
    holonomic_constraint->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

    holonomic_constraint->setVertex(0, last_vertex);
    holonomic_constraint->setVertex(1, new_vertex);
    optimizer_.addEdge(holonomic_constraint);

    latest_vertex_id_++;
    latest_pose_vertex_id_ = latest_vertex_id_;

    return new_vertex;
}

void addIMUEdge(const sensor_msgs::Imu &msg){
    static ros::Time last_time = msg.header.stamp;
    static int last_id = -1;

    g2o::VertexRobotState* last_imu_vertex = nullptr;

    if(last_id == -1) {
        if(print_output) {ROS_INFO_STREAM("last_id == -1. Adding vertex with id " << latest_pose_vertex_id_);}
        last_imu_vertex = static_cast<g2o::VertexRobotState*>(optimizer_.vertex(latest_pose_vertex_id_));
    } else {
        if(print_output) {ROS_INFO_STREAM("last_id == " << last_id << ". Adding vertex with id " << latest_pose_vertex_id_);}
        last_imu_vertex = static_cast<g2o::VertexRobotState*>(optimizer_.vertex(latest_pose_vertex_id_));
    }

    double new_node_from_start = (msg.header.stamp - starting_time_).toSec();
    double last_imu_from_start = last_imu_vertex->estimate().delta_t();
    double delta_t = new_node_from_start - last_imu_from_start;

    g2o::RobotState copy{last_imu_vertex->estimate()};
    copy = copy.withTwist(copy.twist(), delta_t);
    auto* new_vertex = addVertex(copy);
    last_id = new_vertex->id();

    auto imu_edge = new g2o::EdgeIMU();
    g2o::IMUMeasurement measurement{ msg.linear_acceleration.x, msg.angular_velocity.z, delta_t };
    imu_edge->setMeasurement(measurement);
    imu_edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
    imu_edge->setVertex(0, last_imu_vertex);
    imu_edge->setVertex(1, new_vertex);

    optimizer_.addEdge(imu_edge);
}

void setupG2o()
{
    // defining some typedefs
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> MyBlockSolver;
    typedef g2o::LinearSolverCSparse<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // setup the solver
    optimizer_.setVerbose(false);
    auto* solver = new g2o::OptimizationAlgorithmGaussNewton (
            g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));
    optimizer_.setAlgorithm(solver);
}

void setupGraph()
{
    starting_time_ = ros::Time::now();
    g2o::RobotState starting_state{};
    auto* starting_pose = new g2o::VertexRobotState();
    starting_pose->setId(0);
    starting_pose->setEstimate(starting_state);
    starting_pose->setFixed(true);
    optimizer_.addVertex(starting_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");
    ros::NodeHandle pnh {"~"};
    pnh.getParam("print_output", print_output);
    ros::Subscriber odom_sub = pnh.subscribe("/wheel_odom", 1, OdomCallback);
    ros::Subscriber imu_sub = pnh.subscribe("/imu", 1, ImuCallback);
    //ros::Subscriber imu_sub = pnh.subscribe("/velodyne_points", 1, PointCloudCallback);
    setupG2o();
    setupGraph();
    g2o::OptimizableGraph::initMultiThreading();
    ros::spin();
}
