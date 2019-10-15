#include <omp.h>
#include <unordered_set>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <parameter_assertions/assertions.h>

#include "octomapper.h"

using radians = double;
Octomapper::Octomapper(const ros::NodeHandle& pNh)
{
  assertions::getParam(pNh, "octree/resolution", octree_options_.resolution);

  assertions::getParam(pNh, "octree/clamping/max", octree_options_.max);
  assertions::getParam(pNh, "octree/clamping/min", octree_options_.min);

  assertions::getParam(pNh, "map/length", map_options_.length);
  assertions::getParam(pNh, "map/width", map_options_.width);
  assertions::getParam(pNh, "map/start_x", map_options_.start_x);
  assertions::getParam(pNh, "map/start_y", map_options_.start_y);
  assertions::getParam(pNh, "map/log_odds_default", map_options_.default_logodds);
  std::string map_encoding;

  map_options_.resolution = octree_options_.resolution;

  map_encoding_ = CV_8UC1;
}

void Octomapper::createOctree(pc_map_pair &pair) const
{
  pair.octree = boost::make_shared<octomap::OcTree>(octree_options_.resolution);
  pair.octree->setClampingThresMin(octree_options_.min);
  pair.octree->setClampingThresMax(octree_options_.max);
  pair.octree->enableChangeDetection(true);
  octomap::point3d min(-map_options_.length / 2.0, -map_options_.width / 2.0, -1);
  octomap::point3d max(map_options_.length / 2.0, map_options_.width / 2.0, -1);
  pair.octree->setBBXMin(min);
  pair.octree->setBBXMax(max);
}

void PCL_to_Octomap(const pcl::PointCloud<pcl::PointXYZ> &pcl, octomap::Pointcloud &octo)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(pcl, pc2);
  octomap::pointCloud2ToOctomap(pc2, octo);
}

octomap::point3d PCL_to_OctomapPoint(const pcl::PointXYZ &pcl)
{
  return { pcl.x, pcl.y, pcl.z };
}

uchar toCharProb(float p)
{
  return static_cast<uchar>(p * 255);
}

float fromLogOdds(float log_odds)
{
  return 1 - (1 / (1 + exp(log_odds)));
}

std::pair<int, int> Octomapper::toMapCoordinates(double x, double y) const
{
  int x_map = static_cast<int>((map_options_.start_x + x) / octree_options_.resolution);
  int y_map = static_cast<int>((map_options_.start_y + y) / octree_options_.resolution);
  return std::make_pair(x_map, y_map);
}

void Octomapper::getUpdatedMap(struct pc_map_pair &pc_map_pair) const
{
  if (pc_map_pair.map == nullptr)
  {
    create_map(pc_map_pair);
  }

  // Traverse entire tree
  std::vector<std::vector<float>> odds_sum(
      static_cast<unsigned long>(map_options_.lengthGrid()),
      std::vector<float>(static_cast<unsigned long>(map_options_.widthGrid()), map_options_.default_logodds));
  for (octomap::OcTree::iterator it = pc_map_pair.octree->begin(), end = pc_map_pair.octree->end(); it != end; ++it)
  {
    // If this is a leaf at max depth, then only update that node
    if (it.getDepth() == pc_map_pair.octree->getTreeDepth())
    {
      auto [x, y] = toMapCoordinates(it.getX(), it.getY());
      if (x >= 0 && x < map_options_.lengthGrid() && y >= 0 && y < map_options_.widthGrid())
      {
        odds_sum[x][y] += it->getLogOdds();
      }
      else
      {
        ROS_ERROR_STREAM_THROTTLE_NAMED(10, "Point Outside", "Point outside! (" << x << ", " << y << ")");
      }
    }
    else
    {
      // This isn't a leaf at max depth. Time to iterate
      int grid_num = 1u << (pc_map_pair.octree->getTreeDepth() - it.getDepth());
      auto [x, y] = toMapCoordinates(it.getX(), it.getY());
      for (int dx = 0; dx < grid_num; dx++)
      {
        for (int dy = 0; dy < grid_num; dy++)
        {
          if (x + dx >= 0 && x + dx < map_options_.lengthGrid() && y + dy >= 0 && y + dy < map_options_.widthGrid())
          {
            odds_sum[x + dx][y + dy] += it->getLogOdds();
          }
          else
          {
            ROS_ERROR_STREAM_THROTTLE_NAMED(10, "Point Outside", "Point outside!");
          }
        }
      }
    }
  }

  // Transfer from log odds to normal probability
  for (int i = 0; i < map_options_.lengthGrid(); i++)
  {
    for (int j = 0; j < map_options_.widthGrid(); j++)
    {
      pc_map_pair.map->at<uchar>(i, j) = toCharProb(fromLogOdds(odds_sum[i][j]));
    }
  }
}

void Octomapper::createMap(pc_map_pair &pair) const
{
  int length = static_cast<int>(map_options_.lengthGrid());
  int width = static_cast<int>(map_options_.widthGrid());
  pair.map = boost::make_shared<cv::Mat>(length, width, map_encoding_, 127);
}

void Octomapper::insertScan(const tf::Point &sensor_pos, struct pc_map_pair &pair, const PointCloud &pc,
                            ProbabilityModel model, double range) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::point3d sensor = octomap::pointTfToOctomap(sensor_pos);
  sensor.z() = 0;
  octomap::Pointcloud octo_cloud;
  PCL_to_Octomap(pc, octo_cloud);
  pair.octree->insertPointCloud(octo_cloud, sensor, range, false, false);  // No discretize, no lazy eval

  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}

void Octomapper::insertRays(const tf::Point &sensor_pos, pc_map_pair &pair, const PointCloud &pc, bool occupied,
                            ProbabilityModel model) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::point3d sensor = octomap::pointTfToOctomap(sensor_pos);
  sensor.z() = 0;
  octomap::Pointcloud scan;
  PCL_to_Octomap(pc, scan);

  octomap::KeySet keyset{};

  for (const auto &p : scan)
  {
    octomap::KeyRay keyray;
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

void Octomapper::insertRaysWithStartPoint(pc_map_pair &pair, const std::vector<Ray> &pc_rays, bool occupied,
                                          ProbabilityModel model) const
{
  // TODO(Oswin): Change this to RAII like thing
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();
  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::KeySet keyset{};

  for (const Ray &ray : pc_rays)
  {
    octomap::point3d start = PCL_to_OctomapPoint(ray.start);
    octomap::point3d end = PCL_to_OctomapPoint(ray.end);
    octomap::KeyRay keyray;
    if (pair.octree->computeRayKeys(start, end, keyray))
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

  for (const auto &p : octo_cloud)
  {
    octomap::OcTreeKey key;
    if (pair.octree->coordToKeyChecked(p, key))
    {
      pair.octree->updateNode(key, occupied, false);  // lazy_eval = false
    }
  }
  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}

void Octomapper::insertPoints(struct pc_map_pair &pair, const PointCloud &occupied_pc, const PointCloud &free_pc,
                              ProbabilityModel model) const
{
  double old_prob_hit = pair.octree->getProbHit();
  double old_prob_miss = pair.octree->getProbMiss();

  pair.octree->setProbHit(model.prob_hit);
  pair.octree->setProbMiss(model.prob_miss);

  octomap::Pointcloud octo_occupied;
  PCL_to_Octomap(occupied_pc, octo_occupied);
  octomap::KeySet occupied_keyset{};

  for (const auto &p : octo_occupied)
  {
    octomap::OcTreeKey key;
    if (pair.octree->coordToKeyChecked(p, key))
    {
      occupied_keyset.insert(key);
    }
  }

  octomap::Pointcloud octo_free;
  PCL_to_Octomap(free_pc, octo_free);
  octomap::KeySet free_keyset{};

  for (const auto &p : octo_free)
  {
    octomap::OcTreeKey key;
    if (pair.octree->coordToKeyChecked(p, key))
    {
      free_keyset.insert(key);
    }
  }

  // Remove all free from occupied keyset
  for (auto it = free_keyset.begin(), end = free_keyset.end(); it != end;)
  {
    if (occupied_keyset.find(*it) != occupied_keyset.end())
    {
      it = free_keyset.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // Insert
  for (const auto & it : free_keyset)
  {
    pair.octree->updateNode(it, false, false);
  }
  for (const auto & it : occupied_keyset)
  {
    pair.octree->updateNode(it, true, false);
  }

  pair.octree->setProbHit(old_prob_hit);
  pair.octree->setProbMiss(old_prob_miss);
}
