#include <pcl_ros/transforms.h>
#include "particle_filter.h"
#include <tf/transform_broadcaster.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <octomap_ros/conversions.h>
#include <std_msgs/Float64.h>
#include <csignal>

Particle_filter::Particle_filter(const ros::NodeHandle &pNh) : pNh(pNh), m_octomapper(pNh) {
  ros::NodeHandle nh; // Can I do this or do I need to pass it in?

  float resample_threshold;
  igvc::getParam(pNh, "particle_filter/num_particles", m_num_particles);
  igvc::getParam(pNh, "particle_filter/resample_threshold", resample_threshold);
  igvc::getParam(pNh, "particle_filter/variance/x", m_variance_x);
  igvc::getParam(pNh, "particle_filter/variance/y", m_variance_y);
  igvc::getParam(pNh, "particle_filter/variance/yaw", m_variance_yaw);
  igvc::getParam(pNh, "visualization/hue/start", m_viz_hue_start);
  igvc::getParam(pNh, "visualization/hue/end", m_viz_hue_end);
  igvc::getParam(pNh, "visualization/saturation/start", m_viz_sat_start);
  igvc::getParam(pNh, "visualization/saturation/end", m_viz_sat_end);
  igvc::getParam(pNh, "visualization/lightness/start", m_viz_light_start);
  igvc::getParam(pNh, "visualization/lightness/end", m_viz_light_end);
  igvc::getParam(pNh, "debug", m_debug);

  m_inverse_resample_threshold = 1 / (resample_threshold * m_num_particles);

  ROS_INFO_STREAM("Num Particles: " << m_num_particles);
  if (m_debug) {
    m_particle_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    m_ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/ground_debug", 1);
    m_nonground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/nonground_debug", 1);
    m_num_eff_particles_pub = nh.advertise<std_msgs::Float64>("/particle_filter/num_effective_particles", 1);

    if (m_viz_hue_start > m_viz_hue_end) {
      m_viz_hue_end += m_viz_hue_max;
    }
  }
}

void Particle_filter::initialize_particles(const tf::Transform &pose) {
  // Wait for first EKF?
  m_particles.reserve(static_cast<unsigned long>(m_num_particles));
  for (int i = 0; i < m_num_particles; ++i) {
    Particle p;
    p.state.transform = pose;
    m_octomapper.create_octree(p.pair);
    m_particles.emplace_back(std::move(p));
  }
}

//void Particle_filter::visualize_key(const octomap::KeySet freeSet, const octomap::KeySet occSet, const pc_map_pair& pair, uint64_t stamp)
//{
//  pcl::PointCloud<pcl::PointXYZRGB> debug_pcl=
//      pcl::PointCloud<pcl::PointXYZRGB>();
//  for (const auto& free : freeSet)
//  {
//    pcl::PointXYZRGB p;
//    octomap::point3d point = pair.octree->keyToCoord(free);
//    p.x = point.x(); // TODO: Changes these to use start_x
//    p.y = point.y();
//    p.z = point.z();
//    p.r = 0;
//    p.g = 255;
//    p.b = 255;
//    debug_pcl.points.push_back(p);
//  }
//  debug_pcl.header.frame_id = "/odom";
//  debug_pcl.header.stamp = stamp;
//  fuckFree.publish(debug_pcl);
//
//  pcl::PointCloud<pcl::PointXYZRGB> debug_pcl2=
//      pcl::PointCloud<pcl::PointXYZRGB>();
//  for (const auto& occ : occSet)
//  {
//    pcl::PointXYZRGB p;
//    octomap::point3d point = pair.octree->keyToCoord(occ);
//    p.x = point.x(); // TODO: Changes these to use start_x
//    p.y = point.y();
//    p.z = point.z();
//    p.r = 255;
//    p.g = 0;
//    p.b = 255;
//    debug_pcl2.points.push_back(p);
//  }
//  debug_pcl2.header.frame_id = "/odom";
//  debug_pcl2.header.stamp = stamp;
//  fuckOcc.publish(debug_pcl2);
//}

void Particle_filter::update(const tf::Transform& diff, const geometry_msgs::TwistWithCovariance& twist, const ros::Duration delta_t,
                             const pcl::PointCloud<pcl::PointXYZ> &pc, const tf::Transform &lidar_to_base) {
  // Create noise distribution from covariance to sample from
//  for (int i = 0; i < 6; ++i) {
//    ROS_INFO_STREAM(covariance[6 * i] << ", " << covariance[6 * i + 1] << ", " << covariance[6 * i + 2] << ", "
//                                      << covariance[6 * i + 3] << ", " << covariance[6 * i + 4] << ", "
//                                      << covariance[6 * i + 5] << ", ");
//  }
//  static Normal_random_variable uncertainty{m_variance_x, m_variance_y, m_variance_yaw};

  // Separate pc to ground and nonground
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonground = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonground_projected = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
  m_octomapper.filter_ground_plane(pc, *ground, *nonground, coefficients);

  // Project nonground particles on the ground
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  proj.setInputCloud(nonground);
  proj.setModelCoefficients(coefficients);
  proj.filter(*nonground_projected);

  float highest_weight = 0;
  float lowest_weight = 0;
  size_t highest_weight_idx = 0;
  float weight_sum = 0;
  // For each particle in particles
//  ROS_INFO_STREAM("1");
  static tf::TransformBroadcaster br;
//  ROS_INFO_STREAM("(" << twist.twist.linear.x << ", " << twist.twist.linear.y << ", " << twist.twist.linear.z << "), dt: " << delta_t.toSec());
//  ROS_INFO_STREAM("("<<(delta_t.toSec() * twist.twist.linear.x) << ", " << delta_t.toSec() * twist.twist.linear.y << ", " << delta_t.toSec() * twist.twist.linear.z << ")");
//  ROS_INFO_STREAM("transform: " << diff.getOrigin().x() << ", " <<  diff.getOrigin().y() << ", " <<  diff.getOrigin().z() << ")");
//  ROS_INFO_STREAM("Delta t: " << delta_t.toSec());
  for (size_t i = 0; i < m_particles.size(); ++i) {
    // TODO: Add Scanmatching
    // TODO: Add CUDA or OpenMP?
    // Sample new particle from old using pose and covariance
//    ROS_INFO_STREAM("diff: " << diff.getOrigin().x() << ", " << diff.getOrigin().y() << ", " << diff.getOrigin().z());
    double noisy_x = twist.twist.linear.x + gauss(m_variance_x);
    double noisy_y = twist.twist.linear.y + gauss(m_variance_y);
    double cur_yaw = m_particles[i].state.yaw();
    double new_x = noisy_x * cos(cur_yaw) - noisy_y * sin(cur_yaw);
    double new_y = noisy_x * sin(cur_yaw) + noisy_y * cos(cur_yaw);
    m_particles[i].state.set_x(m_particles[i].state.x() + delta_t.toSec() * new_x);
    m_particles[i].state.set_y(m_particles[i].state.y() + delta_t.toSec() * new_y);
    m_particles[i].state.set_yaw(m_particles[i].state.yaw() + delta_t.toSec() * (twist.twist.angular.z + gauss(m_variance_yaw)));
//    ROS_INFO_STREAM("2");

    // Transform particles from base_frame to odom_frame
    pcl::PointCloud<pcl::PointXYZ> transformed_pc;
    pcl_ros::transformPointCloud(pc, transformed_pc, m_particles[i].state.transform); // TODO: Inverse?
//    ROS_INFO_STREAM("3");

    // Transform ground and nonground from base_frame to odom_frame
    // TODO: Is transform faster or plane detection faster? Do I move the ground filtering into the for loop?
    pcl::PointCloud<pcl::PointXYZ> transformed_ground, transformed_nonground;
    pcl_ros::transformPointCloud(*ground, transformed_ground, m_particles[i].state.transform);
    pcl_ros::transformPointCloud(*nonground_projected, transformed_nonground, m_particles[i].state.transform);
//    ROS_INFO_STREAM("4");
    transformed_ground.header.frame_id = "/odom";
    transformed_pc.header.frame_id = "/odom";
    transformed_nonground.header.frame_id = "/odom";
    transformed_ground.header.stamp = pc.header.stamp;
    transformed_pc.header.stamp = pc.header.stamp;
    transformed_nonground.header.stamp = pc.header.stamp;

//    m_ground_pub.publish(transformed_ground);
//    m_nonground_pub.publish(transformed_nonground);

    octomap::KeySet free, occupied;
    tf::Transform odom_to_lidar = m_particles[i].state.transform * lidar_to_base;
    odom_to_lidar.setOrigin(tf::Vector3(odom_to_lidar.getOrigin().x(), odom_to_lidar.getOrigin().y(), 0));

    m_octomapper.separate_occupied(free, occupied, odom_to_lidar.getOrigin(), m_particles[i].pair, transformed_ground,
                                   transformed_nonground);
    // ========================START OF DEBUG=========================================================
//    visualize_key(free, occupied, m_particles[i].pair, pc.header.stamp);
//    octomap::Pointcloud nonground_octo;
//    sensor_msgs::PointCloud2 pc2;
//    pcl::toROSMsg(transformed_nonground, pc2);
//    octomap::pointCloud2ToOctomap(pc2, nonground_octo);
//    octomap::point3d origin = octomap::pointTfToOctomap(odom_to_lidar.getOrigin());
//    m_particles[i].pair.octree->insertPointCloud(nonground_octo, origin, -1, false);
    // =======================END OF DEBUG========================================================

    if (transformed_nonground.size() > 0)
    {
      // Calculate weight using sensor model
      m_particles[i].weight = m_octomapper.sensor_model(m_particles[i].pair, free, occupied);
//    ROS_INFO_STREAM("Weight of " << i << " : " << m_particles[i].weight);
      // Look for lowest weight particle to fix negative weights
    }
    if (m_particles[i].weight < lowest_weight) {
      lowest_weight = m_particles[i].weight;
    }

    // Update map
    m_octomapper.insert_scan(m_particles[i].pair, free, occupied);
  }
  // Fix negative weights
  for (Particle &p : m_particles)
  {
    p.weight -= lowest_weight;
    if (p.weight > highest_weight) {
      highest_weight = p.weight;
    }
    weight_sum += p.weight;
  }

  // Move sum of weights to m_total_weights
  m_total_weight = weight_sum;
  // calculate Neff
  float inverse_n_eff = 0;
  float squared_weight_sum = weight_sum * weight_sum;
  for (Particle &p : m_particles) {
    inverse_n_eff += (p.weight * p.weight) / squared_weight_sum;
  }
  // if Neff < thresh then resample
  if (inverse_n_eff > m_inverse_resample_threshold) {
    if (m_debug) {
      ROS_INFO_STREAM("Performing resampling. N_eff: " << inverse_n_eff << " / " << m_inverse_resample_threshold);
    }
    resample_particles();
  } else {
    ROS_INFO_STREAM("N_eff: " << 1/(m_num_particles*inverse_n_eff) << " / " << 1/(m_num_particles*m_inverse_resample_threshold));
  }

  for (int i = 0; i < m_particles.size(); ++i)
  {
    if (m_particles[i].weight == highest_weight)
    {
      m_best_idx = i;
    }
  }
  // Publish effective particles if debug
  if (m_debug)
  {
    std_msgs::Float64 num;
    num.data = 1/(m_num_particles*inverse_n_eff);
    m_num_eff_particles_pub.publish(num);
  }

  // Update map of best particle for use
  m_octomapper.get_updated_map(m_particles[m_best_idx].pair);

  // Debug publish all particles
  if (m_debug) {
    visualization_msgs::MarkerArray marker_arr;
    int i = 0;
    for (const Particle &particle : m_particles) {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale.x = 0.2;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.header.frame_id = "/odom";
      marker.header.stamp = ros::Time::now();
      double r, g, b;
      marker.pose.position.x = static_cast<float>(particle.state.x());
      marker.pose.position.y = static_cast<float>(particle.state.y());
      marker.pose.position.z = static_cast<float>(particle.state.z());
      marker.pose.orientation.x = particle.state.quat_x();
      marker.pose.orientation.y = particle.state.quat_y();
      marker.pose.orientation.z = particle.state.quat_z();
      marker.pose.orientation.w = particle.state.quat_w();

      // Map weight to rgb using hsl for colorful markers
      map_weight_to_rgb(particle.weight, &r, &g, &b);
//      ROS_INFO_STREAM("(" << r << ", " << g << ", " << b << ")");

      marker.color.r = static_cast<float>(r);
      marker.color.g = static_cast<float>(g);
      marker.color.b = static_cast<float>(b);
      if (fabs(particle.weight - highest_weight) < 1.0e-8) {
        marker.color.r = 1.0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
      }
      marker.id = i++;

      marker_arr.markers.emplace_back(marker);
    }
    m_particle_pub.publish(marker_arr);
  }
}


void Particle_filter::resample_particles() {
  // Create array of cumulative weights
  std::vector<double> cum_weights;
  cum_weights.reserve(static_cast<unsigned long>(m_num_particles));
  cum_weights.emplace_back(m_particles[0].weight);
  for (int i = 1; i < m_num_particles; i++) {
    cum_weights.emplace_back(cum_weights[i - 1] + m_particles[i].weight);
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cum_weights[m_num_particles - 1] / m_num_particles;
  std::uniform_real_distribution<double> unif(0, pointer_width);
  std::random_device rd;
  std::default_random_engine re{rd()};
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  std::vector<struct Particle> sampled_particles;
  sampled_particles.reserve(static_cast<unsigned long>(m_num_particles));
  for (int i = 0; i < m_num_particles; i++) {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width) {
      index++;
    }
    // Found cum_weights[index] >= stating_pointer + i * pointer_width, add that point to the array
    Particle p{};
    p.weight = m_particles[index].weight;
    p.state = m_particles[index].state;
    p.pair = pc_map_pair{};
    p.pair.octree = std::unique_ptr<octomap::OcTree>(new octomap::OcTree(*m_particles[index].pair.octree));
//    octomap::OcTreeNode root = octomap::OcTreeNode(*m_particles[index].pair.octree->getRoot());
//    ROS_INFO_STREAM(m_particles[index].pair.octree->getRoot()->hasChildren() << " == " << root.hasChildren());
//    *p.pair.octree->getRoot() = root;
    sampled_particles.emplace_back(std::move(p));
  }
  // Return sampled array
  m_particles.swap(sampled_particles);
}

double map_range(double inp_start, double inp_end, double out_start, double out_end, double inp) {
  return (inp - inp_start) * (out_end - out_start) / (inp_end - inp_start) + out_start;
}

// Do a proper map function
void Particle_filter::map_weight_to_rgb(float weight, double *r, double *g, double *b) {
  double h = map_range(0, m_total_weight, m_viz_hue_start, m_viz_hue_end, weight);
  h = h > m_viz_hue_max ? h - m_viz_hue_max : h;
  double s = map_range(0, m_total_weight, m_viz_sat_start, m_viz_sat_end, weight);
  s = s > m_viz_sat_max ? s - m_viz_sat_max : s;
  double l = map_range(0, m_total_weight, m_viz_light_start, m_viz_light_end, weight);
  l = l > m_viz_light_max ? l - m_viz_light_max : l;
  hsluv2rgb(h, s, l, r, g, b);
}

double Particle_filter::gauss(double variance) {
  static std::mt19937 gen{ std::random_device{}() };
  std::normal_distribution<> dist(0, variance);

  return dist(gen);
}

