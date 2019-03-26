#include "octomapper.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <omp.h>
#include <unordered_set>

using radians = double;
Octomapper::Octomapper(ros::NodeHandle pNh) : m_default_projection{ 0, 0, 1, 0 }  // Default plane is z = 0
{
  igvc::getParam(pNh, "octree/resolution", m_octree_resolution);
  igvc::getParam(pNh, "sensor_model/hit", m_prob_hit);
  igvc::getParam(pNh, "sensor_model/miss", m_prob_miss);
  igvc::getParam(pNh, "sensor_model/miss_empty", m_prob_miss_empty);
  igvc::getParam(pNh, "sensor_model/min", m_thresh_min);
  igvc::getParam(pNh, "sensor_model/max", m_thresh_max);
  igvc::getParam(pNh, "sensor_model/max_range", m_max_range);
  igvc::getParam(pNh, "map_floor_threshold", m_floor_thresh);
  igvc::getParam(pNh, "ground_filter/enable", m_use_ground_filter);
  igvc::getParam(pNh, "ground_filter/iterations", m_ransac_iterations);
  igvc::getParam(pNh, "ground_filter/distance_threshold", m_ransac_distance_threshold);
  igvc::getParam(pNh, "ground_filter/eps_angle", m_ransac_eps_angle);
  igvc::getParam(pNh, "ground_filter/plane_distance", m_ground_filter_plane_dist);

  igvc::getParam(pNh, "segmented_free_space/kernel_size", m_segmented_kernel);
  igvc::getParam(pNh, "segmented_free_space/std_dev", m_segmented_sigma);
  igvc::getParam(pNh, "segmented_free_space/threshold", m_segmented_threshold);

  igvc::getParam(pNh, "map/length", m_map_length);
  igvc::getParam(pNh, "map/width", m_map_width);
  igvc::getParam(pNh, "map/log_odds_default", m_odds_sum_default);
  std::string map_encoding;
  igvc::getParam(pNh, "map/encoding", map_encoding);
  if (map_encoding == "CV_8UC1")
  {
    m_map_encoding = CV_8UC1;
  }
  else
  {
    m_map_encoding = CV_8UC1;
  }

  m_debug_pub = pNh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/octomapper/projection", 1);
}

void Octomapper::create_octree(pc_map_pair &pair) const
{
  pair.octree = boost::make_shared<octomap::OcTree>(m_octree_resolution);
  pair.octree->setProbHit(m_prob_hit);
  pair.octree->setProbMiss(m_prob_miss);
  pair.octree->setClampingThresMin(m_thresh_min);
  pair.octree->setClampingThresMax(m_thresh_max);
  pair.octree->enableChangeDetection(true);
  octomap::point3d min(-m_map_length / 2.0, -m_map_width / 2.0, -1);
  octomap::point3d max(m_map_length / 2.0, m_map_width / 2.0, -1);
  pair.octree->setBBXMin(min);
  pair.octree->setBBXMax(max);
}

void PCL_to_Octomap(const pcl::PointCloud<pcl::PointXYZ> &pcl, octomap::Pointcloud &octo)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pcl, pc2);
  octomap::pointCloud2ToOctomap(pc2, octo);
}

uchar toCharProb(float p)
{
  return static_cast<uchar>(p * 255);
}

float fromLogOdds(float log_odds)
{
  return 1 - (1 / (1 + exp(log_odds)));
}

std::string key_to_string(octomap::OcTreeKey key)
{
  std::ostringstream out;
  out << std::hex << "(" << key.k[0] << ", " << key.k[1] << ", " << key[2] << ")";
  return out.str();
}

void Octomapper::get_updated_map(struct pc_map_pair &pc_map_pair) const
{
  if (pc_map_pair.map == nullptr)
  {
    create_map(pc_map_pair);
  }

  // Traverse entire tree
  std::vector<std::vector<float>> odds_sum(
      static_cast<unsigned long>(m_map_length / m_octree_resolution),
      std::vector<float>(static_cast<unsigned long>(m_map_width / m_octree_resolution),
                         m_odds_sum_default));  // TODO: Are these the right
  for (octomap::OcTree::iterator it = pc_map_pair.octree->begin(), end = pc_map_pair.octree->end(); it != end; ++it)
  {
    // If this is a leaf at max depth, then only update that node
    if (it.getDepth() == pc_map_pair.octree->getTreeDepth())
    {
      int x = static_cast<int>((m_map_length / 2 + it.getX()) / m_octree_resolution);
      int y = static_cast<int>((m_map_width / 2 + it.getY()) / m_octree_resolution);
      if (x < m_map_length / m_octree_resolution && y < m_map_width / m_octree_resolution)
      {
        odds_sum[x][y] += it->getLogOdds();
      }
      else
      {
        ROS_ERROR_STREAM("Point outside!");
      }
    }
    else
    {
      // This isn't a leaf at max depth. Time to iterate
      int grid_num = 1 << (pc_map_pair.octree->getTreeDepth() - it.getDepth());
      int x = static_cast<int>((m_map_length / 2 + it.getX()) / m_octree_resolution);
      int y = static_cast<int>((m_map_width / 2 + it.getY()) / m_octree_resolution);
      //      ROS_INFO("We did it?");
      for (int dx = 0; dx < grid_num; dx++)
      {
        for (int dy = 0; dy < grid_num; dy++)
        {
          if (x < m_map_length / m_octree_resolution && y < m_map_width / m_octree_resolution)
          {
            odds_sum[x + dx][y + dy] += it->getLogOdds();
          }
          else
          {
            ROS_ERROR_STREAM("Point outside!");
          }
        }
      }
    }
  }

  // Transfer from log odds to normal probability
  for (int i = 0; i < m_map_length / m_octree_resolution; i++)
  {
    for (int j = 0; j < m_map_width / m_octree_resolution; j++)
    {
      pc_map_pair.map->at<uchar>(i, j) = toCharProb(fromLogOdds(odds_sum[i][j]));
    }
  }
}

void Octomapper::create_map(pc_map_pair &pair) const
{
  int length = static_cast<int>(m_map_length / m_octree_resolution);
  int width = static_cast<int>(m_map_width / m_octree_resolution);
  pair.map = boost::make_shared<cv::Mat>(length, width, m_map_encoding, 127);
}

void Octomapper::insert_scan(const tf::Point &sensor_pos_tf, struct pc_map_pair &pc_map_pair,
                             const PCL_point_cloud &raw_pc, const pcl::PointCloud<pcl::PointXYZ> &empty_pc)
{
  pcl::PointCloud<pcl::PointXYZ> ground;
  pcl::PointCloud<pcl::PointXYZ> nonground;
  if (m_use_ground_filter)
  {
    filterGroundPlane(raw_pc, ground, nonground);
  }
  else
  {
    nonground = raw_pc;
  }

  //  ROS_INFO("Filtered Ground");
  octomap::point3d sensor_pos = octomap::pointTfToOctomap(sensor_pos_tf);

  // Convert from PCL_point_cloud to octomap point cloud
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(nonground, octo_cloud);
  octomap::Pointcloud octo_cloud_free;
  PCL_to_Octomap(empty_pc, octo_cloud_free);

  // Insert occupied into octree
  pc_map_pair.octree->insertPointCloud(octo_cloud, sensor_pos, m_max_range, false, false);
  insert_free(octo_cloud_free, sensor_pos, pc_map_pair, false);

  if (m_use_ground_filter)
  {
    octomap::KeySet free_cells;
    octomap::KeyRay key_ray;
    octomap::OcTreeKey end_key;

    for (auto &it : ground)
    {
      octomap::point3d point(it.x, it.y, it.z);

      // If outside max range, set to max range in same direction
      if ((m_max_range > 0.0) && ((point - sensor_pos).norm() > m_max_range))
      {
        point = sensor_pos + (point - sensor_pos).normalized() * m_max_range;
      }

      if (pc_map_pair.octree->computeRayKeys(sensor_pos, point, key_ray))
      {
        free_cells.insert(key_ray.begin(), key_ray.end());
      }

      if (pc_map_pair.octree->coordToKeyChecked(point, end_key))
      {
        //  updateMinKey?? updateMaxKey??
      }
      else
      {
        ROS_ERROR_STREAM("Could not generate Key for endpoint" << point);
      }
    }
    //  ROS_INFO("Done with finding all ground nodes");
    for (const auto &free_cell : free_cells)
    {
      pc_map_pair.octree->updateNode(free_cell, false);
    }
  }
  //  ROS_INFO("Done with inserting ground");
}

void Octomapper::insert_free(const octomap::Pointcloud &scan, octomap::point3d origin, pc_map_pair &pair,
                             bool lazy_eval) const
{
  octomap::KeySet free_cells{};
  pair.octree->setProbMiss(m_prob_miss_empty);

  //  omp_set_num_threads(m_openmp_threads);
  //#pragma omp parallel for schedule(guided)
  for (int i = 0; i < (int)scan.size(); ++i)
  {
    octomap::KeyRay keyray;
    const octomap::point3d &p = scan[i];
    if (pair.octree->computeRayKeys(origin, p, keyray))
    {
      //#pragma omp critical(free_insert)
      {
        free_cells.insert(keyray.begin(), keyray.end());
      }
    }
  }

  for (const auto &free_cell : free_cells)
  {
    pair.octree->updateNode(free_cell, false, lazy_eval);
  }
  pair.octree->setProbMiss(m_prob_miss);
}

void Octomapper::insert_camera_free(struct pc_map_pair &projection_map_pair, const cv::Mat &image,
                                    const image_geometry::PinholeCameraModel &model,
                                    const tf::Transform &camera_to_world) const
{
  // Gaussian blur
  cv::GaussianBlur(image, image, cv::Size(m_segmented_kernel, m_segmented_kernel), m_segmented_sigma,
                   m_segmented_sigma);

  // Threshold
  cv::threshold(image, image, m_segmented_threshold, UCHAR_MAX, cv::THRESH_BINARY);

  pcl::PointCloud<pcl::PointXYZ> projected_pc;
  // If use ground_filter, ie. 3D lidar, then use the coeffs from the last RANSAC. Else, just yolo using current state.
  if (m_use_ground_filter)
  {
    // Check if projection exists
    if (!m_ground_projection.is_defined)
    {
      return;
    }
    projectToPlane(projected_pc, m_ground_projection, image, model, camera_to_world);
  }
  else
  {
    projectToPlane(projected_pc, m_default_projection, image, model, camera_to_world);
  }
  projected_pc.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  projected_pc.header.frame_id = "odom";

  insertCameraProjection(projection_map_pair, projected_pc, false);
  m_debug_pub.publish(projected_pc);
}

/**
 * Inserts a pointcloud representing the projection of the lines detected from the NN onto the lidar scan.
 * @param projection_map_pair Pair consisting of the octree for camera projections and the map
 * @param raw_pc pointcloud from projecting the lines onto the lidar scan
 */
void Octomapper::insertCameraProjection(struct pc_map_pair &projection_map_pair,
                                        const pcl::PointCloud<pcl::PointXYZ> &raw_pc, bool occupied) const
{
  // Project to z=0 plane
  pcl::PointCloud<pcl::PointXYZ> projected{};
  for (auto point : raw_pc.points)
  {
    projected.points.emplace_back(pcl::PointXYZ(point.x, point.y, 0));
  }

  // Convert from PCL_point_cloud to octomap point cloud
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(projected, octo_cloud);

  // Insert occupied into octree
  // TODO: OpenMP?
  octomap::KeySet occupied_cells{};

  for (int i = 0; i < (int)octo_cloud.size(); ++i)
  {
    const octomap::point3d &p = octo_cloud[i];
    octomap::OcTreeKey key;
    if (projection_map_pair.octree->coordToKeyChecked(p, key))
    {
      occupied_cells.insert(key);
    }
  }

  // Insert KeySet into octree
  for (const auto &occupied_cell : occupied_cells)
  {
    projection_map_pair.octree->updateNode(occupied_cell, occupied, false);  // lazy_eval = false
  }
}

void Octomapper::insertScan(const tf::Point &sensor_pos, struct pc_map_pair &pair, const PointCloud &pc,
                            ProbabilityModel model) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::point3d sensor = octomap::pointTfToOctomap(sensor_pos);
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(pc, octo_cloud);
  pair.octree->insertPointCloud(octo_cloud, sensor, m_max_range, false, false);

  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}

void Octomapper::insertRays(const tf::Point &sensor_pos, struct pc_map_pair &pair, const PointCloud &pc, bool occupied,
                            ProbabilityModel model) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::point3d sensor = octomap::pointTfToOctomap(sensor_pos);
  octomap::Pointcloud scan;
  PCL_to_Octomap(pc, scan);

  octomap::KeySet keyset{};

  for (int i = 0; i < (int)scan.size(); ++i)
  {
    octomap::KeyRay keyray;
    const octomap::point3d &p = scan[i];
    if (pair.octree->computeRayKeys(sensor, p, keyray))
    {
      {
        keyset.insert(keyray.begin(), keyray.end());
      }
    }
  }

  for (const auto &key : keyset)
  {
    pair.octree->updateNode(key, occupied, false);
  }

  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}

void Octomapper::insertPoints(struct pc_map_pair &pair, const PointCloud &pc, bool occupied,
                              ProbabilityModel model) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(pc, octo_cloud);
  octomap::KeySet keyset{};

  for (int i = 0; i < (int)octo_cloud.size(); ++i)
  {
    const octomap::point3d &p = octo_cloud[i];
    octomap::OcTreeKey key;
    if (pair.octree->coordToKeyChecked(p, key))
    {
      pair.octree->updateNode(key, occupied, false);  // lazy_eval = false
    }
  }
  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}
