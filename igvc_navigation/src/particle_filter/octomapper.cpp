#include "octomapper.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

// TODO: How to convert OcTree to occupancy grid?
Octomapper::Octomapper(ros::NodeHandle pNh) {
  igvc::getParam(pNh, "octree/resolution", m_octree_resolution);
  igvc::getParam(pNh, "octree/lazy_evaluation", m_lazy_eval);
  igvc::getParam(pNh, "sensor_model/hit", m_prob_hit);
  igvc::getParam(pNh, "sensor_model/miss", m_prob_miss);
  igvc::getParam(pNh, "sensor_model/mismatch_penalty", m_penalty);
  igvc::getParam(pNh, "sensor_model/empty_coeff", m_sensor_empty_coeff);
  igvc::getParam(pNh, "sensor_model/min", m_thresh_min);
  igvc::getParam(pNh, "sensor_model/max", m_thresh_max);
  igvc::getParam(pNh, "sensor_model/max_range", m_max_range);
  igvc::getParam(pNh, "ground_filter/iterations", m_ransac_iterations);
  igvc::getParam(pNh, "ground_filter/distance_threshold", m_ransac_distance_threshold);
  igvc::getParam(pNh, "ground_filter/eps_angle", m_ransac_eps_angle);
  igvc::getParam(pNh, "ground_filter/plane_distance", m_ground_filter_plane_dist);
  igvc::getParam(pNh, "map/length_x", m_map_length);
  igvc::getParam(pNh, "map/width_y", m_map_width);
  igvc::getParam(pNh, "map/start_x", m_map_start_x);
  igvc::getParam(pNh, "map/start_y", m_map_start_y);
  igvc::getParam(pNh, "map/log_odds_default", m_odds_sum_default);
  std::string map_encoding;
  igvc::getParam(pNh, "map/encoding", map_encoding);

  if (map_encoding == "CV_8UC1") {
    m_map_encoding = CV_8UC1;
  } else {
    m_map_encoding = CV_8UC1;
  }

  m_prob_hit_logodds = to_logodds(static_cast<float>(m_prob_hit));
  m_prob_miss_logodds = to_logodds(static_cast<float>(m_prob_miss));

  // TODO: replace all length/resolution with length_grid
  m_map_length_grid = static_cast<int>(m_map_length / m_octree_resolution);
  m_map_width_grid = static_cast<int>(m_map_width / m_octree_resolution);

#ifdef _OPENMP
#pragma omp parallel
#pragma omp critical
  {
    if (omp_get_thread_num() == 0) {
      ROS_INFO_STREAM("OMP threads: " << omp_get_num_threads());
      m_keyrays.resize(omp_get_num_threads());
    }
  }
#else
  m_keyrays.resize(1);
#endif
}

void Octomapper::create_octree(pc_map_pair &pair) const {
  pair.octree = boost::make_shared<octomap::OcTree>(m_octree_resolution);
  pair.octree->setProbHit(m_prob_hit);
  pair.octree->setProbMiss(m_prob_miss);
  pair.octree->setClampingThresMin(m_thresh_min);
  pair.octree->setClampingThresMax(m_thresh_max);
  pair.octree->enableChangeDetection(true);
  octomap::point3d min(static_cast<float>(-m_map_length / 2.0), static_cast<float>(-m_map_width / 2.0), -1);
  octomap::point3d max(static_cast<float>(m_map_length / 2.0), static_cast<float>(m_map_width / 2.0), -1);
  pair.octree->setBBXMin(min);
  pair.octree->setBBXMax(max);
}

void PCL_to_Octomap(const pcl::PointCloud<pcl::PointXYZ> &pcl, octomap::Pointcloud &octo) {
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pcl, pc2);
  octomap::pointCloud2ToOctomap(pc2, octo);
}

uchar toCharProb(float p) {
  return static_cast<uchar>(p * 255);
}

float fromLogOdds(float log_odds) {
  return 1 - (1 / (1 + exp(log_odds)));
}

std::string key_to_string(octomap::OcTreeKey key) {
  std::ostringstream out;
  out << std::hex << "(" << key.k[0] << ", " << key.k[1] << ", " << key[2] << ")";
  return out.str();
}

void Octomapper::get_updated_map(struct pc_map_pair &pc_map_pair) const {
  if (pc_map_pair.map == nullptr) {
    create_map(pc_map_pair);
  }
  // TODO: Think about how to serialize / deserialize efficiently maybe?
  // for (auto it = pc_map_pair.octree->changedKeysBegin(); it != pc_map_pair.octree->changedKeysEnd(); ++it)
  //{
  //  // TODO: Do we want the lowest depth here?
  //  octomap::OcTreeNode* node = pc_map_pair.octree->search(it->first, 0);
  //  auto coord = pc_map_pair.octree->keyToCoord(it->first);
  //  int x = coord.x() * m_octree_resolution;
  //  int y = coord.y() * m_octree_resolution;
  //}

  // Traverse entire tree
  double minX, minY, minZ, maxX, maxY, maxZ;
  pc_map_pair.octree->getMetricMin(minX, minY, minZ);
  pc_map_pair.octree->getMetricMax(maxX, maxY, maxZ);

  octomap::point3d minPt(minX, minY, minZ);
  octomap::point3d maxPt(maxX, maxY, maxZ);
  //  ROS_INFO_STREAM("Min: " << minPt << ", Max: " << maxPt);

  octomap::OcTreeKey minKey = pc_map_pair.octree->coordToKey(minPt);
  octomap::OcTreeKey maxKey = pc_map_pair.octree->coordToKey(maxPt);
  minX = std::min(minX, -0.5 * m_map_length);
  maxX = std::max(maxX, 0.5 * m_map_length);
  minY = std::min(minY, -0.5 * m_map_width);
  maxY = std::max(maxY, 0.5 * m_map_width);
  minPt = octomap::point3d(minX, minY, minZ);
  maxPt = octomap::point3d(maxX, maxY, maxZ);

  octomap::OcTreeKey padded_min_key, padded_max_key;
  int depth = pc_map_pair.octree->getTreeDepth();
  if (!pc_map_pair.octree->coordToKeyChecked(minPt, static_cast<unsigned int>(depth), padded_min_key)) {
    ROS_ERROR_STREAM("Could not create padded min OcTree key at " << minPt);
  }
  if (!pc_map_pair.octree->coordToKeyChecked(maxPt, static_cast<unsigned int>(depth), padded_max_key)) {
    ROS_ERROR_STREAM("Could not create padded max OcTree key at " << maxPt);
  }

  //  ROS_INFO_STREAM("Min: " << minPt << ", Max: " << maxPt);
  //  ROS_INFO_STREAM("MinKey: " << key_to_string(minKey) << ", Max: " << key_to_string(maxKey));
  std::vector<std::vector<float>> odds_sum(m_map_length_grid,
                                           std::vector<float>(m_map_width_grid,
                                                              m_odds_sum_default));  // TODO: Are these the right
  for (octomap::OcTree::iterator it = pc_map_pair.octree->begin(), end = pc_map_pair.octree->end(); it != end; ++it) {
    // If this is a leaf at max depth, then only update that node
    if (it.getDepth() == pc_map_pair.octree->getTreeDepth()) {
      //      ROS_INFO_STREAM("it.getKey(): " << key_to_string(it.getKey()) << "  coords: (" << it.getX() << ", " <<
      //      it.getY() << ", " << it.getZ() << ")"); ROS_INFO_STREAM("Resolution: " << m_octree_resolution << "  pad: "
      //      << m_map_length/2);
      int x = (m_map_length / 2 + it.getX()) / m_octree_resolution;
      int y = (m_map_width / 2 + it.getY()) / m_octree_resolution;
      //      ROS_INFO_STREAM("Sum: (" << x << ", " << y << ")");
      if (x < m_map_length / m_octree_resolution && y < m_map_width / m_octree_resolution) {
        odds_sum[x][y] += it->getLogOdds();
      } else {
        ROS_ERROR_STREAM("Point outside!");
      }
    } else {
      // This isn't a leaf at max depth. Time to iterate
      int grid_num = 1 << (pc_map_pair.octree->getTreeDepth() - it.getDepth());
      octomap::OcTreeKey minKey = it.getIndexKey();
      int x = static_cast<int>((m_map_length / 2 + it.getX()) / m_octree_resolution);
      int y = static_cast<int>((m_map_width / 2 + it.getY()) / m_octree_resolution);
      //      ROS_INFO("We did it?");
      for (int dx = 0; dx < grid_num; dx++) {
        for (int dy = 0; dy < grid_num; dy++) {
          odds_sum[x + dx][y + dy] += it->getLogOdds();  // TODO: Which direction do I add??
        }
      }
    }
  }

  // Transfer from log odds to normal probability
  for (int i = 0; i < m_map_length / m_octree_resolution; i++) {
    for (int j = 0; j < m_map_width / m_octree_resolution; j++) {
      pc_map_pair.map->at<uchar>(i, j) = toCharProb(fromLogOdds(odds_sum[i][j]));
    }
  }
}

void Octomapper::create_map(pc_map_pair &pair) const {
  pair.map = boost::make_shared<cv::Mat>(m_map_length_grid, m_map_width_grid, m_map_encoding, 127);
}

void Octomapper::insert_scan(struct pc_map_pair &pc_map_pair, octomap::KeySet &free_cells,
                             octomap::KeySet &occupied_cells) const {
  for (const auto &free_cell : free_cells) {
    pc_map_pair.octree->updateNode(free_cell, false);
  }
  for (const auto &occupied_cell : occupied_cells) {
    pc_map_pair.octree->updateNode(occupied_cell, true);
  }
}

void Octomapper::filter_ground_plane(const PCL_point_cloud &raw_pc, PCL_point_cloud &ground,
                                     PCL_point_cloud &nonground) const {
  //  ROS_INFO_STREAM("Filtering Ground with " << raw_pc.size() << " points");
  ground.header = raw_pc.header;
  nonground.header = raw_pc.header;

  if (raw_pc.size() < 50) {
    // Hacky algorithm to detect ground
    ROS_ERROR_STREAM("Pointcloud while filtering too small, skipping" << raw_pc.size());
    nonground = raw_pc;
  } else {
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

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
      ROS_INFO("PCL segmentation did not find a plane.");
    } else {
      if (std::abs(coefficients->values.at(3)) < m_ransac_distance_threshold) {
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                  cloud_filtered->size(), coefficients->values.at(0), coefficients->values.at(1),
                  coefficients->values.at(2), coefficients->values.at(3));
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(ground);

        // remove ground points from full pointcloud
        if (inliers->indices.size() != cloud_filtered->size()) {
          extract.setNegative(true);
          PCL_point_cloud out;
          extract.filter(out);
          nonground += out;
          *cloud_filtered = out;
        }
        ground_plane_found = true;
      } else {
        ROS_INFO("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                 cloud_filtered->size(), coefficients->values.at(0), coefficients->values.at(1),
                 coefficients->values.at(2), coefficients->values.at(3));
      }
    }
    //    ROS_INFO_STREAM("Cloud_filtered Points: " << cloud_filtered->size());
    //    ROS_INFO_STREAM("ground points: " << ground.size());
    //    ROS_INFO_STREAM("nonground points: " << nonground.size());
    if (!ground_plane_found) {
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

void Octomapper::separate_occupied(octomap::KeySet &free_cells, octomap::KeySet &occupied_cells,
                                   const tf::Point &sensor_pos_tf, const pc_map_pair &pair,
                                   const pcl::PointCloud<pcl::PointXYZ> &ground,
                                   const pcl::PointCloud<pcl::PointXYZ> &nonground) {
  octomap::point3d origin = octomap::pointTfToOctomap(sensor_pos_tf);

  octomap::Pointcloud nonground_octo;
  PCL_to_Octomap(nonground, nonground_octo);
  octomap::Pointcloud ground_octo;
  PCL_to_Octomap(ground, ground_octo);

  // Separate all voxels touched by ground and nongroudn into free and occupied
  compute_voxels(*pair.octree, nonground_octo, ground_octo, origin, free_cells, occupied_cells);
}

float Octomapper::sensor_model(const pc_map_pair &pair, const octomap::KeySet &free_cells,
                               const octomap::KeySet &occupied_cells) const {
  float total = 1; // So the total is greater than 0
  // TODO: OPENMP?
  for (const auto &free_cell : free_cells) {
    octomap::OcTreeNode *leaf = pair.octree->search(free_cell);
    if (leaf) {
      // Can check sign bit using bit operators, but need to benchmark
      total += leaf->getLogOdds() < 0 ? leaf->getLogOdds() * m_sensor_empty_coeff * m_prob_miss_logodds : leaf->getLogOdds() * m_penalty * m_prob_miss_logodds;
    }
  }
  for (const auto &occupied_cell : occupied_cells) {
    octomap::OcTreeNode *leaf = pair.octree->search(occupied_cell);
    if (leaf) {
      total += leaf->getLogOdds() > 0 ? leaf->getLogOdds() * m_prob_hit_logodds : leaf->getLogOdds() * m_penalty * m_prob_hit_logodds;
    }
  }
  return total;
}

void Octomapper::compute_voxels(const octomap::OcTree &tree, const octomap::Pointcloud &scan,
                                const octomap::Pointcloud &ground, const octomap::point3d &origin,
                                octomap::KeySet &free_cells, octomap::KeySet &occupied_cells) {
#ifdef _OPENMP
  omp_set_num_threads(m_keyrays.size());
#endif
#pragma omp parallel
  {
    // Project all nonground
    #pragma omp for
    for (int i = 0; i < scan.size(); ++i) {
      const octomap::point3d &p = scan[i];
      unsigned threadIdx = 0;
      #ifdef _OPENMP
      threadIdx = omp_get_thread_num();
      #endif
      octomap::KeyRay *keyray = &(m_keyrays.at(threadIdx));
      if (tree.computeRayKeys(origin, p, *keyray)) {
        #pragma omp critical
        {
          free_cells.insert(keyray->begin(), keyray->end());
          octomap::OcTreeKey key;
          if (tree.coordToKeyChecked(p, key)) {
            occupied_cells.insert(key);
          }
        }
      }
    }
  }

  // Add all ground
  for (const auto &p : ground) {
    octomap::KeyRay keyray;
    if (tree.computeRayKeys(origin, p, keyray)) {
//      ROS_INFO("I am called");
      free_cells.insert(keyray.begin(), keyray.end());
    }
  }

  // Remove from free if in occupied
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end;) {
    if (occupied_cells.find(*it) != occupied_cells.end()) {
      it = free_cells.erase(it);
    } else {
      ++it;
    }
  }
}
