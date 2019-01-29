#include "scanmatcher.h"
#include <pcl/common/transforms.h>
#include <igvc_utils/RobotState.hpp>
#include <tf/transform_broadcaster.h>

struct Scored_move
{
  RobotState pose;
  double score;
  double likelihood;
};

Scanmatcher::Scanmatcher(const ros::NodeHandle &pNh, const Octomapper &octomapper)
  : m_last_cloud{ nullptr }, m_octomapper{ octomapper }
{
  ros::NodeHandle nh;
  igvc::getParam(pNh, "scanmatcher/use_guess", m_use_guess);
  igvc::getParam(pNh, "scanmatcher/icp/iterations", m_icp_iterations);
  igvc::getParam(pNh, "scanmatcher/icp/RANSAC_outlier_rejection_threshold", m_RANSAC_outlier_rejection);
  igvc::getParam(pNh, "scanmatcher/icp/max_correspondence_distance", m_correspondence_distance);
  igvc::getParam(pNh, "scanmatcher/icp/transformation_epsilon", m_transformation_epsilon);
  igvc::getParam(pNh, "scanmatcher/optimizer/angular_delta_start", m_a_delta_start);
  igvc::getParam(pNh, "scanmatcher/optimizer/linear_delta_start", m_l_delta_start);
  igvc::getParam(pNh, "scanmatcher/optimizer/max_refinements", m_max_refinements);
  m_prev_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/previous_cloud", 1);
  m_last_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/last_cloud", 1);
  m_next_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/scanmatcher/next_cloud", 1);
}

double Scanmatcher::scanmatch(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, tf::Transform &transform,
                              const tf::Transform &guess)
{
  m_prev_pub.publish(*input);
  m_last_pub.publish(*m_last_cloud);
  // TODO: Combine multiple point clouds for a better scanmatch?
  double fitness_score = -1;
  if (m_last_cloud != nullptr)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(m_last_cloud);
    icp.setInputTarget(input);
    icp.setMaximumIterations(m_icp_iterations);
    icp.setRANSACOutlierRejectionThreshold(m_RANSAC_outlier_rejection);
    icp.setMaxCorrespondenceDistance(m_correspondence_distance);
    icp.setTransformationEpsilon(m_transformation_epsilon);

    pcl::PointCloud<pcl::PointXYZ> final;

    icp.align(final);
    if (!icp.hasConverged())
    {
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
    tf3d.setValue(m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1), m(2, 2));
    transform.setOrigin(origin);
    transform.setBasis(tf3d);
  }
  m_last_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*input);
  return fitness_score;
}

double Scanmatcher::icp(tf::Transform &optimized_transform, const pc_map_pair &pair, const tf::Transform &guess,
                        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) const
{
}

double Scanmatcher::optimize(tf::Transform &optimized_transform, const pc_map_pair &pair, const tf::Transform &guess,
                             const pcl::PointCloud<pcl::PointXYZ> &cloud) const
{
  double best_score = -1;
  RobotState current_pose{ guess };
  std::vector<Scored_move> move_list;
  move_list.reserve(64);
  Scored_move sm = { RobotState{ guess }, 0, 0 };

  octomap::KeySet occupied;
  double current_score = m_octomapper.get_score(*pair.octree, cloud, guess);
//  ROS_INFO_STREAM("Starting score: " << current_score << ", starting pose: " << current_pose);
//  ros::Duration(0.2).sleep();
  sm.score = current_score;

  move_list.emplace_back(sm);
  double a_delta = m_a_delta_start;
  double l_delta = m_l_delta_start;
  int iterations = 0;
  int refinements = 0;
  enum Move
  {
    Forward,
    Back,
    Left,
    Right,
    CW,
    CCW,
    Done
  };
  RobotState best_local_pose{ current_pose };
  static tf::TransformBroadcaster br;
  do
  {
//    ROS_INFO_STREAM("best score: " << best_score << ", current score:" << current_score << ", best pose: " << current_pose);
    if (best_score >= current_score)
    {
      refinements++;
      a_delta *= 0.5;
      l_delta *= 0.5;
    }
    best_score = current_score;
    RobotState local_pose{ current_pose };
    Move move = Forward;
    do
    {
      local_pose = current_pose;
      switch (move)
      {
        case Forward:
          local_pose.forward(l_delta);
//          ROS_INFO_STREAM("fwd: " << local_pose);
          move = Back;
          break;
        case Back:
//          ROS_INFO_STREAM("back: " << local_pose);
          local_pose.forward(-l_delta);
          move = Left;
          break;
        case Left:
          local_pose.left(l_delta);
          move = Right;
          break;
        case Right:
          local_pose.left(-l_delta);
          move = CCW;
          break;
        case CCW:
//          ROS_INFO_STREAM("ccw: " << local_pose);
          local_pose.turn_ccw(a_delta);
          move = CW;
          break;
        case CW:
//          ROS_INFO_STREAM("cw: " << local_pose);
          local_pose.turn_ccw(-a_delta);
          move = Done;
          break;
        case Done:
        default:;
      }
      // TODO: odo_gain?
      double local_score = m_octomapper.get_score(*pair.octree, cloud, local_pose.transform);
//      ros::Duration(0.1).sleep();
      iterations++;
      if (local_score > current_score)
      {
        current_score = local_score;
        best_local_pose = local_pose;
      }
      sm.score = local_score;
      sm.likelihood = local_score;  // TODO: What is likelihood?
      sm.pose = local_pose;
      move_list.emplace_back(sm);
    } while (move != Done);
//    br.sendTransform(
//        tf::StampedTransform(best_local_pose.transform, ros::Time::now(), "/odom", "/scanmatch"));
//  ROS_INFO_STREAM(current_score << " --> (" << best_local_pose.transform.getOrigin().x() << ", "
//                             << best_local_pose.transform.getOrigin().y() << ")");
    current_pose = best_local_pose;
  } while (current_score > best_score || refinements < m_max_refinements);
  optimized_transform = best_local_pose.transform;
  double r, p, y;
  optimized_transform.getBasis().getRPY(r, p, y);
//  ROS_INFO_STREAM(best_score << " --> (" << optimized_transform.getOrigin().x() << ", "
//                             << optimized_transform.getOrigin().y() << ", " << y << ") in " << iterations
//                             << " iterations with " << refinements << " refinements");
  return best_score;
}
