#include "octomapper.h"

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

// TODO: How to convert OcTree to occupancy grid?
Octomapper::Octomapper(ros::NodeHandle pNh) {
  igvc::getParam(pNh, "octree/resolution", m_octree_resolution);
  igvc::getParam(pNh, "sensor_model/hit", m_prob_hit);
  igvc::getParam(pNh, "sensor_model/miss", m_prob_miss);
  igvc::getParam(pNh, "sensor_model/min", m_thresh_min);
  igvc::getParam(pNh, "sensor_model/max", m_thresh_max);
  igvc::getParam(pNh, "sensor_model/max_range", m_max_range);
  igvc::getParam(pNh, "ground_filter/iterations", m_ransac_iterations);
  igvc::getParam(pNh, "ground_filter/distance_threshold", m_ransac_distance_threshold);
  igvc::getParam(pNh, "ground_filter/eps_angle", m_ransac_eps_angle);
  igvc::getParam(pNh, "ground_filter/plane_distance", m_ground_filter_plane_dist);
  igvc::getParam(pNh, "map/length", m_map_length);
  igvc::getParam(pNh, "map/width", m_map_width);
  igvc::getParam(pNh, "map/log_odds_default", m_odds_sum_default);
  std::string map_encoding;
  igvc::getParam(pNh, "map/encoding", map_encoding);
  if (map_encoding == "CV_8UC1") {
    m_map_encoding = CV_8UC1;
  } else {
    m_map_encoding = CV_8UC1;
  }
}

void Octomapper::create_octree(pc_map_pair &pair) const {
  pair.octree = boost::make_shared<octomap::OcTree>(m_octree_resolution);
  pair.octree->setProbHit(m_prob_hit);
  pair.octree->setProbMiss(m_prob_miss);
  pair.octree->setClampingThresMin(m_thresh_min);
  pair.octree->setClampingThresMax(m_thresh_max);
  pair.octree->enableChangeDetection(true);
  octomap::point3d min(-m_map_length / 2, -m_map_width / 2, -1);
  octomap::point3d max(m_map_length / 2, m_map_width / 2, -1);
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
  if (!pc_map_pair.octree->coordToKeyChecked(minPt, depth, padded_min_key)) {
    ROS_ERROR_STREAM("Could not create padded min OcTree key at " << minPt);
  }
  if (!pc_map_pair.octree->coordToKeyChecked(maxPt, depth, padded_max_key)) {
    ROS_ERROR_STREAM("Could not create padded max OcTree key at " << maxPt);
  }

//  ROS_INFO_STREAM("Min: " << minPt << ", Max: " << maxPt);
//  ROS_INFO_STREAM("MinKey: " << key_to_string(minKey) << ", Max: " << key_to_string(maxKey));
  std::vector<std::vector<float>> odds_sum(m_map_length / m_octree_resolution,
                                           std::vector<float>(m_map_width / m_octree_resolution,
                                                              m_odds_sum_default));  // TODO: Are these the right
  for (octomap::OcTree::iterator it = pc_map_pair.octree->begin(), end = pc_map_pair.octree->end(); it != end; ++it) {
    // If this is a leaf at max depth, then only update that node
    if (it.getDepth() == pc_map_pair.octree->getTreeDepth()) {
//      ROS_INFO_STREAM("it.getKey(): " << key_to_string(it.getKey()) << "  coords: (" << it.getX() << ", " << it.getY() << ", " << it.getZ() << ")");
//      ROS_INFO_STREAM("Resolution: " << m_octree_resolution << "  pad: " << m_map_length/2);
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
      int x = (m_map_length / 2 + it.getX()) / m_octree_resolution;
      int y = (m_map_width / 2 + it.getY()) / m_octree_resolution;
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
  int length = m_map_length / m_octree_resolution;
  int width = m_map_width / m_octree_resolution;
  pair.map = boost::make_shared<cv::Mat>(length, width, m_map_encoding, 127);
}

void Octomapper::insert_scan(const tf::Point &sensor_pos_tf, struct pc_map_pair &pc_map_pair,
                             const Octomapper::PCL_point_cloud &raw_pc) const {
//  ROS_INFO("Inserting scan");
//  ROS_INFO_STREAM(
//      "Sensor position: " << sensor_pos_tf.x() << ", " << sensor_pos_tf.y() << ", " << sensor_pos_tf.z() << ")");
//  ROS_INFO_STREAM("Arbitrary cloud point" << raw_pc.at(0));

  pcl::PointCloud<pcl::PointXYZ> within_range;
//  filter_range(raw_pc, within_range);
  pcl::PointCloud<pcl::PointXYZ> ground;
  pcl::PointCloud<pcl::PointXYZ> nonground;
  filter_ground_plane(raw_pc, ground, nonground);
//  ROS_INFO("Filtered Ground");
  octomap::point3d sensor_pos = octomap::pointTfToOctomap(sensor_pos_tf);

  // Convert from PCL_point_cloud to octomap point cloud
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(nonground, octo_cloud);
//  ROS_INFO("Inserting pointcloud");
//  ROS_INFO_STREAM("Num points: " << octo_cloud.size());
  // TODO: Do we need to discretize to speed up? Probably not, since non occupied is so small
  pc_map_pair.octree->insertPointCloud(octo_cloud, sensor_pos, m_max_range, false, false);

  octomap::KeySet free_cells;
  octomap::KeyRay key_ray;
  octomap::OcTreeKey end_key;

  for (auto it = ground.begin(); it != ground.end(); ++it) {
    octomap::point3d point(it->x, it->y, it->z);

    // If outside max range, set to max range in same direction
    if ((m_max_range > 0.0) && ((point - sensor_pos).norm() > m_max_range)) {
      point = sensor_pos + (point - sensor_pos).normalized() * m_max_range;
    }

    if (pc_map_pair.octree->computeRayKeys(sensor_pos, point, key_ray)) {
      free_cells.insert(key_ray.begin(), key_ray.end());
    }

    if (pc_map_pair.octree->coordToKeyChecked(point, end_key)) {
      //  updateMinKey?? updateMaxKey??
    } else {
      ROS_ERROR_STREAM("Could not generate Key for endpoint" << point);
    }
  }
//  ROS_INFO("Done with finding all ground nodes");
  for (const auto &free_cell : free_cells) {
    pc_map_pair.octree->updateNode(free_cell, false);
  }
//  ROS_INFO("Done with inserting ground");
  // TODO: When to generate occupancy grid?
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

