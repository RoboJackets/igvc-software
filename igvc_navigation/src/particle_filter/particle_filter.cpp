#include "particle_filter.h"
#include <octomap_ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <csignal>

Particle_filter::Particle_filter(const ros::NodeHandle &pNh)
  : pNh{ pNh }, m_octomapper{ pNh }, m_scanmatcher{ pNh, m_octomapper }
{
  ros::NodeHandle nh;  // Can I do this or do I need to pass it in?

  float resample_threshold;
  igvc::getParam(pNh, "is_3d", m_is_3d);
  igvc::getParam(pNh, "particle_filter/num_particles", m_num_particles);
  igvc::getParam(pNh, "particle_filter/resample_threshold", resample_threshold);
  igvc::getParam(pNh, "particle_filter/variance/x", m_variance_x);
  igvc::getParam(pNh, "particle_filter/variance/y", m_variance_y);
  igvc::getParam(pNh, "particle_filter/variance/yaw", m_variance_yaw);
  igvc::getParam(pNh, "particle_filter/still_threshold", m_thresh_still);
  igvc::getParam(pNh, "scanmatcher/enable", m_use_scanmatch);
  igvc::getParam(pNh, "scanmatcher/point_threshold", m_scanmatch_point_thresh);
  igvc::getParam(pNh, "scanmatcher/variance/x", m_scanmatch_variance_x);
  igvc::getParam(pNh, "scanmatcher/variance/y", m_scanmatch_variance_y);
  igvc::getParam(pNh, "scanmatcher/variance/yaw", m_scanmatch_variance_yaw);
  igvc::getParam(pNh, "visualization/hue/start", m_viz_hue_start);
  igvc::getParam(pNh, "visualization/hue/end", m_viz_hue_end);
  igvc::getParam(pNh, "visualization/saturation/start", m_viz_sat_start);
  igvc::getParam(pNh, "visualization/saturation/end", m_viz_sat_end);
  igvc::getParam(pNh, "visualization/lightness/start", m_viz_light_start);
  igvc::getParam(pNh, "visualization/lightness/end", m_viz_light_end);
  igvc::getParam(pNh, "debug", m_debug);

  m_inverse_resample_threshold = 1 / (resample_threshold * m_num_particles);

  ROS_INFO_STREAM("Num Particles: " << m_num_particles);
  if (m_debug)
  {
    m_particle_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    m_ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/ground_debug", 1);
    m_nonground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/nonground_debug", 1);
    m_scanmatched_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/scanmatched_nonground", 1);
    m_num_eff_particles_pub = nh.advertise<std_msgs::Float64>("/particle_filter/num_effective_particles", 1);

    if (m_viz_hue_start > m_viz_hue_end)
    {
      m_viz_hue_end += m_viz_hue_max;
    }
  }
}

void Particle_filter::initialize_particles(const tf::Transform &pose)
{
  // Wait for first EKF?
  m_particles.reserve(static_cast<unsigned long>(m_num_particles));
  for (int i = 0; i < m_num_particles; ++i)
  {
    Particle p{};
    p.weight = 1;
    p.state.transform = pose;
    m_octomapper.create_octree(p.pair);
    m_particles.emplace_back(std::move(p));
  }
}

// void Particle_filter::visualize_key(const octomap::KeySet freeSet, const octomap::KeySet occSet, const pc_map_pair&
// pair, uint64_t stamp)
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

void Particle_filter::update(const tf::Transform &diff, const geometry_msgs::TwistWithCovariance &twist,
                             const ros::Duration delta_t, const pcl::PointCloud<pcl::PointXYZ> &pc,
                             const tf::Transform &lidar_to_base)
{
  // Set lidar to base transformation if not set
  m_octomapper.set_lidar_to_base(lidar_to_base);

  // Separate pc to ground and nonground
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonground = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonground_projected = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
  if (m_is_3d)
  {
    m_octomapper.filter_ground_plane(pc, *ground, *nonground, coefficients);
  }
  else
  {
    *nonground = pc;
  }

  if (m_debug)
  {
    nonground->header.frame_id = "/base_link";
    m_nonground_pub.publish(*nonground);
    if (m_is_3d)
    {
      ground->header.frame_id = "/base_link";
      m_ground_pub.publish(*ground);
    }
  }

  // Project nonground particles on the ground
  //  pcl::ProjectInliers<pcl::PointXYZ> proj;
  //  proj.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  //  proj.setInputCloud(nonground);
  //  proj.setModelCoefficients(coefficients);
  //  proj.filter(*nonground_projected);


  for (auto &p : nonground->points)
  {
    p.z = 0;
  }
  if (m_is_3d)
  {
    for (auto &p : ground->points)
    {
      p.z = 0;
    }
  }

  float highest_weight = 0;
  double weight_sum = 0;
  // For each particle in particles
  //  ROS_INFO_STREAM("1");
  static tf::TransformBroadcaster br;
  //  ROS_INFO_STREAM("(" << twist.twist.linear.x << ", " << twist.twist.linear.y << ", " << twist.twist.linear.z << "),
  //  dt: " << delta_t.toSec()); ROS_INFO_STREAM("("<<(delta_t.toSec() * twist.twist.linear.x) << ", " <<
  //  delta_t.toSec() * twist.twist.linear.y << ", " << delta_t.toSec() * twist.twist.linear.z << ")");
  //  ROS_INFO_STREAM("transform: " << diff.getOrigin().x() << ", " <<  diff.getOrigin().y() << ", " <<
  //  diff.getOrigin().z() << ")"); ROS_INFO_STREAM("Delta t: " << delta_t.toSec());
  static int iterations = -1;
  iterations++;
//#pragma omp parallel
//#pragma omp for
  for (size_t i = 0; i < m_particles.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ> transformed_ground, transformed_nonground;
    if (m_is_3d)
    {
      pcl_ros::transformPointCloud(*ground, transformed_ground, m_particles[i].state.transform);
    }
    pcl_ros::transformPointCloud(*nonground, transformed_nonground, m_particles[i].state.transform);

    double fitness = -1;
    tf::Transform scanmatch_motion_model;
    if (nonground->size() >= m_scanmatch_point_thresh && m_use_scanmatch && iterations >= 10)
    {
      tf::Transform scanmatch_transform;
      RobotState guess;
      double cur_yaw = m_particles[i].state.yaw();
      double new_x = twist.twist.linear.x * cos(cur_yaw) - twist.twist.linear.y * sin(cur_yaw);
      double new_y = twist.twist.linear.x * sin(cur_yaw) + twist.twist.linear.y * cos(cur_yaw);
      guess.set_x(m_particles[i].state.x() + delta_t.toSec() * new_x);
      guess.set_y(m_particles[i].state.y() + delta_t.toSec() * new_y);
      guess.set_yaw(m_particles[i].state.yaw() + delta_t.toSec() * twist.twist.angular.z);
      nonground->header.frame_id = "/base_link";
//    fitness = m_scanmatcher.scanmatch(nonground, scanmatch_transform, guess);
      fitness = m_scanmatcher.optimize(scanmatch_transform, m_particles[i].pair, guess.transform, *nonground);
      if (fitness != -1)
      {
        scanmatch_motion_model = scanmatch_transform;
        pcl::PointCloud<pcl::PointXYZ> fuck;
        pcl_ros::transformPointCloud(*nonground, fuck, scanmatch_motion_model);
        fuck.header.frame_id = "/odom";
        m_scanmatched_pub.publish(fuck);
      }
      double x = scanmatch_transform.getOrigin().getX();
      double y = scanmatch_transform.getOrigin().getY();
      scanmatch_transform.setOrigin(tf::Vector3(x, y, 0));
      double r, p, yaw;
      tf::Matrix3x3 rota = scanmatch_transform.getBasis();
      rota.getRPY(r, p, yaw);
//      ROS_INFO_STREAM("Fitness: " << fitness << "\t\t Transform: (" << x << ", " << y << ", " << yaw << ")");
      scanmatch_transform.getBasis().setRPY(0, 0, yaw);
    }

    double noisy_x, noisy_y, noisy_yaw;
    // If scanmatching failed or not using scanmatching, then use EKF output
    Particle particle{};
    if (fitness == -1)
    {
      if (iterations < 10 ||
          (fabs(twist.twist.linear.x) < m_thresh_still && fabs(twist.twist.linear.y) < m_thresh_still && fabs(twist.twist.angular.z) < m_thresh_still))
      {
        noisy_x = 0;
        noisy_y = 0;
        noisy_yaw = 0;
      }
      else
      {
        noisy_x = twist.twist.linear.x + gauss(m_variance_x);
        noisy_y = twist.twist.linear.y + gauss(m_variance_y);
        noisy_yaw = twist.twist.angular.z + gauss(m_variance_yaw);
      }
      double cur_yaw = m_particles[i].state.yaw() + delta_t.toSec() * noisy_yaw;
      double new_x = noisy_x * cos(cur_yaw) - noisy_y * sin(cur_yaw);
      double new_y = noisy_x * sin(cur_yaw) + noisy_y * cos(cur_yaw);
      particle.state.set_x(m_particles[i].state.x() + delta_t.toSec() * new_x);
      particle.state.set_y(m_particles[i].state.y() + delta_t.toSec() * new_y);
      particle.state.set_yaw(m_particles[i].state.yaw() + delta_t.toSec() * noisy_yaw);
    }
    else
    {
      particle.state.transform = scanmatch_motion_model;
      particle.state.set_x(particle.state.x() + gauss(m_scanmatch_variance_x));
      particle.state.set_y(particle.state.y() + gauss(m_scanmatch_variance_y));
      particle.state.set_yaw(particle.state.yaw() + gauss(m_scanmatch_variance_yaw));
    }

    // Transform particles from base_frame to odom_frame
    pcl::PointCloud<pcl::PointXYZ> transformed_pc;
    pcl_ros::transformPointCloud(pc, transformed_pc, particle.state.transform);  // TODO: Inverse?
                                                                                 //    ROS_INFO_STREAM("3");

    // Transform ground and nonground from base_frame to odom_frame
    // TODO: Is transform faster or plane detection faster? Do I move the ground filtering into the for loop?
    //    ROS_INFO_STREAM("4");
    //    transformed_ground.header.frame_id = "/odom";
    //    transformed_pc.header.frame_id = "/odom";
    //    transformed_nonground.header.frame_id = "/odom";
    //    transformed_ground.header.stamp = pc.header.stamp;
    //    transformed_pc.header.stamp = pc.header.stamp;
    //    transformed_nonground.header.stamp = pc.header.stamp;

    //    m_ground_pub.publish(transformed_ground);
    //    m_nonground_pub.publish(transformed_nonground);

    octomap::KeySet free, occupied;
    tf::Transform odom_to_lidar = particle.state.transform * lidar_to_base;
    odom_to_lidar.setOrigin(tf::Vector3(odom_to_lidar.getOrigin().x(), odom_to_lidar.getOrigin().y(), 0));
    tf::Matrix3x3 rot = odom_to_lidar.getBasis();
    double r, p, y;
    rot.getRPY(r, p, y);
    rot.setRPY(0, 0, y);

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
      particle.weight *= m_octomapper.sensor_model(m_particles[i].pair, free, occupied);
//#pragma omp atomic update
      weight_sum += particle.weight;
      //    ROS_INFO_STREAM("Weight of " << i << " : " << m_particles[i].weight);
      // Update map
      if (particle.weight > highest_weight)
      {
//#pragma omp atomic write
        highest_weight = particle.weight;
      }
      m_octomapper.insert_scan(m_particles[i].pair, free, occupied);
//#pragma omp atomic write
      m_particles[i].weight = particle.weight;
//#pragma omp critical(update_state)
      m_particles[i].state = std::move(particle.state);
    }
  }
  //  std::stringbuf str;
  //  std::ostream stream(&str);
  //  for (Particle &p : m_particles) {
  //    stream << p.weight << ", ";
  //  }
  //  ROS_INFO_STREAM(str.str());
  if (weight_sum == 0)
  {
    ROS_ERROR_STREAM("Weights somehow became 0. Resetting all to 1");
    for (Particle &p : m_particles)
    {
      p.weight = 1;
    }
    weight_sum = m_particles.size();
    highest_weight = 1;
  }
  if (nonground->size() > 0)
  {
    // Move sum of weights to m_total_weights
    m_total_weight = weight_sum;
    // calculate Neff
    float inverse_n_eff = 0;
    float squared_weight_sum = weight_sum * weight_sum;
    highest_weight /= weight_sum;
    for (Particle &p : m_particles)
    {
      inverse_n_eff += (p.weight * p.weight) / squared_weight_sum;
      p.weight /= weight_sum; // Make sure that particle weights don't keep getting smaller
    }
    // if Neff < thresh then resample
    if (inverse_n_eff > m_inverse_resample_threshold)
    {
      if (m_debug)
      {
        ROS_INFO_STREAM("Performing resampling. N_eff: " << 1 / inverse_n_eff << " / "
                                                         << 1 / m_inverse_resample_threshold);
      }
      resample_particles();
    }
    else
    {
      ROS_INFO_STREAM("N_eff: " << 1 / (inverse_n_eff) << " / " << 1 / (m_inverse_resample_threshold));
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
      num.data = 1 / (inverse_n_eff);
      m_num_eff_particles_pub.publish(num);
    }

    // Update map of best particle for use
    m_octomapper.get_updated_map(m_particles[m_best_idx].pair);
  }

  // Debug publish all particles
  if (m_debug)
  {
    visualization_msgs::MarkerArray marker_arr;
    int i = 0;
    for (const Particle &particle : m_particles)
    {
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
      if (fabs(particle.weight - highest_weight) < 1.0e-8)
      {
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

void Particle_filter::resample_particles()
{
  // Create array of cumulative weights
  std::vector<double> cum_weights;
  cum_weights.reserve(static_cast<unsigned long>(m_num_particles));
  cum_weights.emplace_back(m_particles[0].weight);
  for (int i = 1; i < m_num_particles; i++)
  {
    cum_weights.emplace_back(cum_weights[i - 1] + m_particles[i].weight);
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cum_weights[m_num_particles - 1] / m_num_particles;
  std::uniform_real_distribution<double> unif(0, pointer_width);
  std::random_device rd;
  std::default_random_engine re{ rd() };
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  std::vector<struct Particle> sampled_particles;
  sampled_particles.reserve(static_cast<unsigned long>(m_num_particles));
  for (int i = 0; i < m_num_particles; i++)
  {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width)
    {
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

double map_range(double inp_start, double inp_end, double out_start, double out_end, double inp)
{
  return (inp - inp_start) * (out_end - out_start) / (inp_end - inp_start) + out_start;
}

// Do a proper map function
void Particle_filter::map_weight_to_rgb(float weight, double *r, double *g, double *b)
{
  double h = map_range(0, m_total_weight, m_viz_hue_start, m_viz_hue_end, weight);
  h = h > m_viz_hue_max ? h - m_viz_hue_max : h;
  double s = map_range(0, m_total_weight, m_viz_sat_start, m_viz_sat_end, weight);
  s = s > m_viz_sat_max ? s - m_viz_sat_max : s;
  double l = map_range(0, m_total_weight, m_viz_light_start, m_viz_light_end, weight);
  l = l > m_viz_light_max ? l - m_viz_light_max : l;
  hsluv2rgb(h, s, l, r, g, b);
}

double Particle_filter::gauss(double variance)
{
  static std::mt19937 gen{ std::random_device{}() };
  std::normal_distribution<> dist(0, variance);

  return dist(gen);
}
