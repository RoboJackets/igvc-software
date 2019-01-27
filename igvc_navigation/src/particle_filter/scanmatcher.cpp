#include "scanmatcher.h"
#include <pcl/common/transforms.h>

Scanmatcher::Scanmatcher(const ros::NodeHandle& pNh, const Octomapper& octomapper)
        :m_last_cloud(nullptr), m_octomapper(octomapper)
{
    ros::NodeHandle nh;
    igvc::getParam(pNh, "scanmatcher/use_guess", m_use_guess);
    igvc::getParam(pNh, "scanmatcher/icp/iterations", m_icp_iterations);
    igvc::getParam(pNh, "scanmatcher/icp/RANSAC_outlier_rejection_threshold", m_RANSAC_outlier_rejection);
    igvc::getParam(pNh, "scanmatcher/icp/max_correspondence_distance", m_correspondence_distance);
    igvc::getParam(pNh, "scanmatcher/icp/transformation_epsilon", m_transformation_epsilon);
    m_prev_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/previous_cloud", 1);
    m_last_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/last_cloud", 1);
    m_next_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/next_cloud", 1);
}

double Scanmatcher::scanmatch(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, tf::Transform& transform,
        const tf::Transform& guess)
{
    m_prev_pub.publish(*input);
    m_last_pub.publish(*m_last_cloud);
    // TODO: Combine multiple point clouds for a better scanmatch?
    double fitness_score = -1;
    if (m_last_cloud!=nullptr) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(m_last_cloud);
        icp.setInputTarget(input);
        icp.setMaximumIterations(m_icp_iterations);
        icp.setRANSACOutlierRejectionThreshold(m_RANSAC_outlier_rejection);
        icp.setMaxCorrespondenceDistance(m_correspondence_distance);
        icp.setTransformationEpsilon(m_transformation_epsilon);

        pcl::PointCloud<pcl::PointXYZ> final;

        icp.align(final);
        if (!icp.hasConverged()) {
            m_last_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*input);
            return -1;
        }
        fitness_score = icp.getFitnessScore();
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*m_last_cloud, transformed_cloud, icp.getFinalTransformation());
        transformed_cloud.header.frame_id = "/base_link";
        m_next_pub.publish(final);

        Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();
        tf::Vector3 origin;
        origin.setValue(m(0, 3), m(1, 3), m(2, 3));
        tf::Matrix3x3 tf3d;
        tf3d.setValue(m(0, 0), m(0, 1), m(0, 2),
                m(1, 0), m(1, 1), m(1, 2),
                m(2, 0), m(2, 1), m(2, 2));
        transform.setOrigin(origin);
        transform.setBasis(tf3d);
    }
    m_last_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*input);
    return fitness_score;
}

double Scanmatcher::optimize(tf::Transform& optimized_transform, const pc_map_pair& pair, const tf::Transform& guess,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    double best_score = -1;
    tf::Transform current_pose = guess;
    octomap::KeySet occupied;
    m_octomapper.compute_occupied(*pair.octree, *cloud, )
    double current_score = m_octomapper.sensor_model(pair, nullptr, )
}
