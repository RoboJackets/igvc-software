#include "scanmatcher.h"

double Scanmatcher::scanmatch(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, tf::Transform transform,
                           const tf::Transform& guess)
{
  // TODO: Combine multiple point clouds for a better scanmatch?
  double fitness_score = -1;
  if (m_last_cloud != nullptr)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(m_last_cloud);
    icp.setInputTarget(input);
    pcl::PointCloud<pcl::PointXYZ> final;
    if (m_use_guess)
    {
      Eigen::Affine3d affine_guess;
      tf::transformTFToEigen(guess, affine_guess);
      Eigen::Matrix4d m = affine_guess.matrix();
      icp.align(final, m);
    } else {
      icp.align(final);
    }
    fitness_score = icp.getFitnessScore();

    Eigen::Affine3d affine;
    affine.matrix() = icp.getFinalTransformation().cast<double>();
    tf::transformEigenToTF(affine, transform);
  }
  m_last_cloud = input;
  return fitness_score;
}
