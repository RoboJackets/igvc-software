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
    filter_ground_plane(raw_pc, ground, nonground);
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

void Octomapper::filter_ground_plane(const PCL_point_cloud &raw_pc, PCL_point_cloud &ground, PCL_point_cloud &nonground)
{
  //  ROS_INFO_STREAM("Filtering Ground with " << raw_pc.size() << " points");
  ground.header = raw_pc.header;
  nonground.header = raw_pc.header;

  if (raw_pc.size() < 50)
  {
    // Hacky algorithm to detect ground
    ROS_ERROR_STREAM("Pointcloud while filtering too small, skipping" << raw_pc.size());
    nonground = raw_pc;
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

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
    {
      ROS_INFO("PCL segmentation did not find a plane.");
    }
    else
    {
      ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
                cloud_filtered->size(), coefficients->values.at(0), coefficients->values.at(1),
                coefficients->values.at(2), coefficients->values.at(3));
      m_ground_projection.set(coefficients->values);
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
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
    //    ROS_INFO_STREAM("Cloud_filtered Points: " << cloud_filtered->size());
    //    ROS_INFO_STREAM("ground points: " << ground.size());
    //    ROS_INFO_STREAM("nonground points: " << nonground.size());
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

void Octomapper::insert_camera_free(struct pc_map_pair &projection_map_pair, const cv::Mat &image,
                                    const image_geometry::PinholeCameraModel &model,
                                    const tf::Transform &camera_to_world) const
{
  ROS_INFO_STREAM_THROTTLE(1, "in insert_camera_free");;
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
    project_to_plane(projected_pc, m_ground_projection, image, model, camera_to_world);
  }
  else
  {
    project_to_plane(projected_pc, m_default_projection, image, model, camera_to_world);
  }
  projected_pc.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  projected_pc.header.frame_id = "odom";

  ROS_INFO_THROTTLE(0.5, "final: (%.2f, %.2f, %.2f)", projected_pc.points[0].x, projected_pc.points[0].y, projected_pc.points[0].z);
  insert_camera_projection(projection_map_pair, projected_pc, false);
  m_debug_pub.publish(projected_pc);
}

/**
 * Inserts a pointcloud representing the projection of the lines detected from the NN onto the lidar scan.
 * @param projection_map_pair Pair consisting of the octree for camera projections and the map
 * @param raw_pc pointcloud from projecting the lines onto the lidar scan
 */
void Octomapper::insert_camera_projection(struct pc_map_pair &projection_map_pair,
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

/**
 * Projects all black pixels (0, 0, 0) in the image to the ground plane and inserts them into the pointcloud
 * @param[out] projected_pc the pointcloud that holds all projected points
 * @param[in] m_ground_projection the coefficients of the ground plane
 * @param[in] image the image to project from
 * @param[in] model the camera model to be used for projection
 */
void Octomapper::project_to_plane(pcl::PointCloud<pcl::PointXYZ> &projected_pc, const Ground_plane &m_ground_projection,
                                  const cv::Mat &image, const image_geometry::PinholeCameraModel &model,
                                  const tf::Transform &camera_to_world) const
{
  int nRows = image.rows;
  int nCols = image.cols;

  int i, j;
  const uchar *p;
  for (i = 0; i < nRows; ++i)
  {
    p = image.ptr<uchar>(i);
    for (j = 0; j < nCols; ++j)
    {
      // If it's a white pixel => free space, then project
      if (p[j] == 0)
      {
//        ROS_INFO("(%d, %d)", i, j);
        cv::Point2d pixel_point(j, i);
        double t1 = j - model.cx() - model.Tx();
        double t2 = i - model.cy() - model.Ty();
//        ROS_INFO_THROTTLE(0.1, "=====For (%d, %d)======", j, i);
//        ROS_INFO_THROTTLE(0.1, "(t1, t2) = (%.2f, %.2f)\n", t1, t2);
//        ROS_INFO_THROTTLE(0.1, "(fx(), fy()) = (%.2f, %.2f)\n", model.fx(), model.fy());
        cv::Point3d ray = model.projectPixelTo3dRay(pixel_point);
//        ROS_INFO_STREAM_THROTTLE(0.1, "ray: (" << ray.x << ", " << ray.y << ", " << ray.z << ")");
        tf::Point reoriented_ray{ ray.x, ray.y, ray.z };  // cv::Point3d defined with z forward, x right, y down
//        ROS_INFO_STREAM_THROTTLE(0.1, "reoriented_ray: (" << reoriented_ray.x() << ", " << reoriented_ray.y() << ", " << reoriented_ray.z() << ")");
        tf::Point transformed_ray = camera_to_world.getBasis() * reoriented_ray; // Transform ray to odom frame
//        ROS_INFO_STREAM_THROTTLE(0.1, "transformed_ray: (" << transformed_ray.x() << ", " << transformed_ray.y() << ", " << transformed_ray.z() << ")");
//        double scale = -camera_to_world.getOrigin().z() / transformed_ray.z();
//        tf::Point projected_point = scale * transformed_ray + camera_to_world.getOrigin();
        double a = m_ground_projection.a;
        double b = m_ground_projection.b;
        double c = m_ground_projection.c;
        double d = m_ground_projection.d;
//        ROS_INFO_THROTTLE(1, "%.2f x + %.2f y + %.2f z + %.2f = 0", m_ground_projection.a, m_ground_projection.b, m_ground_projection.c, m_ground_projection.d);
        // [a b c]^T dot (P0 - (camera + ray*t)) = 0, solve for t
        double t = ((tf::Vector3{0, 0, d/c} - camera_to_world.getOrigin()).dot(tf::Vector3{a, b, c}))/(transformed_ray.dot(tf::Vector3{a, b, c}));
        // projected_point = camera + ray*t
        tf::Point projected_point = camera_to_world.getOrigin() + transformed_ray * t;
        projected_pc.points.emplace_back(pcl::PointXYZ(static_cast<float>(projected_point.x()),
                                                       static_cast<float>(projected_point.y()),
                                                       static_cast<float>(projected_point.z())));
      }
    }
  }
}
