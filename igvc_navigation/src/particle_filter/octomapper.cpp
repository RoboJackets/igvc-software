#include "octomapper.h"

Octomapper::Octomapper(ros::NodeHandle pNh)
{
  igvc::getParam(pNh, "sensor_model/hit", m_prob_hit);
  igvc::getParam(pNh, "sensor_model/miss", m_prob_miss);
  igvc::getParam(pNh, "sensor_model/min", m_thresh_min);
  igvc::getParam(pNh, "sensor_model/max", m_thresh_max);
  igvc::getParam(pNh, "ground_filter/iterations", m_ransac_iterations);
  igvc::getParam(pNh, "ground_filter/distance_threshold", m_ransac_distance_threshold);
  igvc::getParam(pNh, "ground_filter/eps_angle", m_ransac_eps_angle);
  igvc::getParam(pNh, "ground_filter/plane_distance", m_ground_filter_plane_dist);
}

void Octomapper::create_octree(boost::shared_ptr<octomap::OcTree> tree)
{
  tree = boost::make_shared<octomap::OcTree>();
  tree->setProbHit(m_prob_hit);
  tree->setProbMiss(m_prob_miss);
  tree->setClampingThresMin(m_thresh_min);
  tree->setClampingThresMax(m_thresh_max);
}

void PCL_to_Octomap(const pcl::PointCloud<pcl::PointXYZ>& pcl, octomap::Pointcloud& octo)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pcl, pc2);
  octomap::pointCloud2ToOctomap(pc2, octo);
}

void Octomapper::insert_scan(const tf::Point& sensor_pos_tf, boost::shared_ptr<octomap::OcTree> map,
                             const pcl::PointCloud<pcl::PointXYZ>& ground,
                             const pcl::PointCloud<pcl::PointXYZ>& nonground) const
{
  octomap::point3d sensor_pos = octomap::pointTfToOctomap(sensor_pos_tf);
  octomap::OcTree a(*map);

  // Convert from PCL_point_cloud to octomap point cloud
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(nonground, octo_cloud);

  // TODO: Do we need to discretize to speed up? Probably not, since non occupied is so small
  map->insertPointCloud(octo_cloud, sensor_pos, m_max_range, false, false);

  octomap::KeySet free_cells;
  octomap::KeyRay key_ray;
  octomap::OcTreeKey end_key;

  for (PCL_point_cloud::const_iterator it = ground.begin(); it != ground.end(); ++it)
  {
    octomap::point3d point(it->x, it->y, it->z);

    // If outside max range, set to max range in same direction
    if ((m_max_range > 0.0) && ((point - sensor_pos).norm() > m_max_range))
    {
      point = sensor_pos + (point - sensor_pos).normalized() * m_max_range;
    }

    if (map->computeRayKeys(sensor_pos, point, key_ray))
    {
      free_cells.insert(key_ray.begin(), key_ray.end());
    }

    if (map->coordToKeyChecked(point, end_key))
    {
      //  updateMinKey?? updateMaxKey??
    }
    else
    {
      ROS_ERROR_STREAM("Could not generate Key for endpoint" << point);
    }
  }
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
  {
    map->updateNode(*it, false);
  }
}

void Octomapper::filter_ground_plane(const PCL_point_cloud& raw_pc, PCL_point_cloud& ground,
                                     PCL_point_cloud& nonground) const
{
  ground.header = raw_pc.header;
  nonground.header = raw_pc.header;

  if (raw_pc.size() < 50)
  {
    // Hacky algorithm to detect ground
  }
  else
  {
    // Plane detection for ground removal
    pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(m_ransac_iterations);  // TODO: How many iterations
    seg.setDistanceThreshold(m_ransac_distance_threshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(m_ransac_eps_angle);

    PCL_point_cloud::Ptr cloud_filtered = raw_pc.makeShared();
    // Create filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    bool ground_plane_found = false;

    while (cloud_filtered->size() > 10 && !ground_plane_found)
    {
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        ROS_INFO("PCL segmentation did not find a plane.");
        break;
      }

      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_ransac_distance_threshold)
      {
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                  cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1),
                  coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative(false);
        extract.filter(ground);

        // remove ground points from full pointcloud
        if (inliers->indices.size() != cloud_filtered->size())
        {
          extract.setNegative(true);
          PCL_point_cloud out;
          extract.filter(out);
          nonground += out;
          *cloud_filtered = out;
        }
        ground_plane_found = true;
      }
      else
      {
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                  cloud_filtered->size(), coefficients->values.at(0), coefficients->values.at(1),
                  coefficients->values.at(2), coefficients->values.at(3));
      }
    }

    if (!ground_plane_found)
    {
      pcl::PassThrough<pcl::PointXYZ> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_ground_filter_plane_dist, m_ground_filter_plane_dist);
      second_pass.setInputCloud(cloud_filtered);
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative(true);
      second_pass.filter(nonground);
    }
  }
}
