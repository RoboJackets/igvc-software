#include <cv_bridge/cv_bridge.h>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/RobotState.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/circular_buffer.hpp>

#include "particle_filter.h"

#include <ros/ros.h>
#include <signal.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class ParticleFilterNode {
public:
  ParticleFilterNode(double transform_max_wait_time, double update_time_thresh, double start_x, double start_y,
                     double start_z, double pc_buf_size, double pose_buf_size, bool debug)
      : m_debug(debug), m_update_time_thresh(update_time_thresh), m_start_x(start_x), m_start_y(start_y),
        m_start_z(start_z), m_pc_buf(pc_buf_size), m_pose_buf(pose_buf_size),
        m_lidar_transform(nullptr) {
  }

  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc);

  void pose_callback(const nav_msgs::OdometryConstPtr &pose);

  tf::TransformListener m_tf_listener;
  std::unique_ptr<Particle_filter> m_particle_filter;
  ros::Publisher m_map_pub;
  ros::Publisher m_map_pcl_debug_pub;
  std::string m_base_frame, m_lidar_frame;
  std::string m_lidar_topic, m_fused_topic, m_odom_frame;

private:
  void check_update();

  void update(int pose_idx, int pc_idx, const ros::Time &stamp);

  void publish(const ros::Time &stamp);

  void get_lidar_transform();

  bool m_debug;
  bool m_initialised = false;
  double m_update_time_thresh, m_start_x, m_start_y, m_start_z;
  std::shared_ptr<tf::Stamped<tf::Pose>> m_last_pose;
  tf::TransformBroadcaster br;
  cv_bridge::CvImage m_img_bridge;
  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> m_pc_buf;
  boost::circular_buffer<nav_msgs::OdometryConstPtr> m_pose_buf;
  boost::shared_ptr<tf::StampedTransform> m_lidar_transform;
};

std::unique_ptr<ParticleFilterNode> pf_node;

void node_cleanup(int sig) {
  pf_node->m_particle_filter.reset();
  pf_node.reset();
  ros::shutdown();
}

void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
  m_pc_buf.push_back(pc);
  check_update();
}

void ParticleFilterNode::pose_callback(const nav_msgs::OdometryConstPtr &pose) {
  m_pose_buf.push_back(pose);
  check_update();
}

/**
 * Compares m_pose_buf and m_pc_buf; Will update with every lidar, but needs to match the correct pose with it
 */
void ParticleFilterNode::check_update() {
//  ROS_INFO_STREAM("pc buf size: " << m_pc_buf.size() << " pose buf size: " << m_pose_buf.size());
  // Needs both buffers to have at least one element
  if (m_pc_buf.empty() || m_pose_buf.empty()) {
    return;
  }

  ros::Time pc_stamp;
  pcl_conversions::fromPCL(m_pc_buf.front()->header.stamp, pc_stamp);

  // If pose stamp > pc_stamp, then update, since the difference will only get larger,
  // and don't want to dump lidar msgs ?? Do we??
  for (size_t i = 0; i < m_pose_buf.size(); ++i) {
    if (pc_stamp - m_pose_buf[i]->header.stamp < ros::Duration(m_update_time_thresh)) {
      update(i, 0, pc_stamp);
      return;
    } else if (m_pose_buf[i]->header.stamp > pc_stamp) {
      update(i, 0, m_pose_buf[i]->header.stamp);
      return;
    }
  }
}

/**
 * Performs one iteration of particle filter using the indices passed
 * @param pose_idx
 * @param pc_idx
 */
void ParticleFilterNode::update(int pose_idx, int pc_idx, const ros::Time &stamp) {
  // Convert nav_msgs pose to tf pose
  tf::Stamped<tf::Pose> cur_pose;
  tf::poseMsgToTF(m_pose_buf[pose_idx]->pose.pose, cur_pose);

  // On first run, initialize particles with the pose
  if (!m_initialised)
  {
    m_particle_filter->initialize_particles(cur_pose);
    m_last_pose = std::make_shared<tf::Stamped<tf::Pose>>(cur_pose);
    m_initialised = true;
  }

  // Get pose difference
  tf::Transform diff = m_last_pose->inverseTimes(cur_pose);
  diff.setRotation(diff.getRotation().normalize());

  // TODO: Maybe move this to a lidar filtering node?
  // Get static transform from lidar frame to base frame
  get_lidar_transform();

  // TODO: Make new node to filter pc instead of doing it here
  pcl::PointCloud<pcl::PointXYZ>::Ptr small(new pcl::PointCloud<pcl::PointXYZ>);
  float radius = 40;
  float distanceFromSphereCenterPoint;
  bool pointIsWithinSphere;
  for (const auto& point_i : *m_pc_buf[pc_idx]) {
    distanceFromSphereCenterPoint = point_i.x * point_i.x +
        point_i.y * point_i.y +
        point_i.z * point_i.z;
    pointIsWithinSphere = distanceFromSphereCenterPoint <= radius;
    if (pointIsWithinSphere) {
      small->push_back(point_i);
    }
  }

  // Transform cloud to base frame
  pcl::PointCloud<pcl::PointXYZ> transformed_pc;
  pcl_ros::transformPointCloud(*small, transformed_pc, *m_lidar_transform);

  //TODO: Do I need to subtract covariances?
  m_particle_filter->update(diff, m_pose_buf[pose_idx]->pose.covariance, transformed_pc, *m_lidar_transform);
//  ROS_INFO("5");

  // Delete buffer till index
  m_pose_buf.erase_begin(static_cast<unsigned long>(pose_idx + 1));
  m_pc_buf.erase_begin(static_cast<unsigned long>(pc_idx + 1));

  // Update last pose
  *m_last_pose = cur_pose;

  // Publish newest iteration of particle filter
  publish(stamp);
//  ROS_INFO("Done with update in particle_filter_node");
}

void ParticleFilterNode::publish(const ros::Time &stamp) {
  // Publish map form best particle
  igvc_msgs::map message;    // >> message to be sent
  sensor_msgs::Image image;  // >> image in the message
  m_img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8,
                                    *m_particle_filter->m_best_particle.pair.map);
  m_img_bridge.toImageMsg(image);  // from cv_bridge to sensor_msgs::Image

  // Setup message
  double resolution = m_particle_filter->resolution();
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = m_particle_filter->length_x();
  message.width = m_particle_filter->width_y();
  message.resolution = resolution;
  message.orientation = m_particle_filter->m_best_particle.state.yaw();
  message.x = std::round(m_particle_filter->m_best_particle.state.x() / resolution) + m_particle_filter->start_x();
  message.y = std::round(m_particle_filter->m_best_particle.state.y() / resolution) + m_particle_filter->start_y();
  message.x_initial = m_particle_filter->start_x();
  message.y_initial = m_particle_filter->start_y();

  // Publish map
  m_map_pub.publish(message);

  if (m_debug)
  {
    uint64 pc_stamp;
    pcl_conversions::toPCL(stamp, pc_stamp);
    pcl::PointCloud<pcl::PointXYZRGB> debug_pcl=
        pcl::PointCloud<pcl::PointXYZRGB>();
    for (int i = 0; i < m_particle_filter->length_x(); i++) {
      for (int j = 0; j < m_particle_filter->width_y(); j++) {
        pcl::PointXYZRGB p;
        uchar prob = m_particle_filter->m_best_particle.pair.map->at<uchar>(i, j);
        if (prob > 127) {
          p = pcl::PointXYZRGB();
          p.x = (i - m_particle_filter->length_x() / 2) * resolution; // TODO: Changes these to use start_x
          p.y = (j - m_particle_filter->width_y() / 2) * resolution;
          p.r = 0;
          p.g = static_cast<uint8_t>((prob - 127) * 2);
          p.b = 0;
          debug_pcl.points.push_back(p);
        } else if (prob < 127) {
          p = pcl::PointXYZRGB();
          p.x = (i - m_particle_filter->length_x() / 2) * resolution;
          p.y = (j - m_particle_filter->width_y() / 2) * resolution;
          p.r = 0;
          p.g = 0;
          p.b = static_cast<uint8_t>((127 - prob) * 2);
          debug_pcl.points.push_back(p);
        }
        // Set x y coordinates as the center of the grid cell.
      }
    }
    debug_pcl.header.frame_id = "/odom";
    debug_pcl.header.stamp = pc_stamp;
//    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
    m_map_pcl_debug_pub.publish(debug_pcl);
  }
}

void ParticleFilterNode::get_lidar_transform() {
  if (m_lidar_transform == nullptr) {
    if (m_tf_listener.waitForTransform(m_base_frame, m_lidar_frame, ros::Time(0), ros::Duration(2))) {
      ROS_INFO_STREAM("Getting static transform from " << m_lidar_frame << " to " << m_base_frame);
      m_lidar_transform = boost::make_shared<tf::StampedTransform>();
      tf::StampedTransform t;
      m_tf_listener.lookupTransform(m_base_frame, m_lidar_frame, ros::Time(0), t);
      *m_lidar_transform = t;
    } else {
      ROS_ERROR_STREAM("Error getting static transform from " << m_lidar_frame << " to " << m_base_frame);
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  signal(SIGINT, node_cleanup);

  double transform_max_wait_time, update_time_threshold;
  double start_x, start_y, start_z;
  double pc_buf_size, pose_buf_size;
  bool debug;
  igvc::getParam(pNh, "transform_max_wait_time", transform_max_wait_time);
  igvc::getParam(pNh, "update_time_threshold", update_time_threshold);
  igvc::getParam(pNh, "start_x", start_x);
  igvc::getParam(pNh, "start_y", start_y);
  igvc::getParam(pNh, "start_z", start_z);
  igvc::getParam(pNh, "particle_filter_buffer_size", pc_buf_size);
  igvc::getParam(pNh, "pose_buffer_size", pose_buf_size);
  igvc::param(pNh, "debug", debug, false);

  pf_node = std::unique_ptr<ParticleFilterNode>(
      new ParticleFilterNode(transform_max_wait_time, update_time_threshold, start_x, start_y, start_z, pc_buf_size,
                             pose_buf_size, debug));
  igvc::getParam(pNh, "lidar_topic", pf_node->m_lidar_topic);
  igvc::getParam(pNh, "fused_topic", pf_node->m_fused_topic);
  igvc::getParam(pNh, "odometry_frame", pf_node->m_odom_frame);
  igvc::getParam(pNh, "base_frame", pf_node->m_base_frame);
  igvc::getParam(pNh, "lidar_frame", pf_node->m_lidar_frame);

  pf_node->m_particle_filter = std::unique_ptr<Particle_filter>(new Particle_filter(pNh));

  ros::Subscriber pc_sub = nh.subscribe(pf_node->m_lidar_topic, 1, &ParticleFilterNode::pc_callback, pf_node.get());
  ros::Subscriber pose_sub = nh.subscribe(pf_node->m_fused_topic, 1, &ParticleFilterNode::pose_callback, pf_node.get());

  pf_node->m_map_pub = nh.advertise<igvc_msgs::map>("/map", 1);
  pf_node->m_map_pcl_debug_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/particle_filter/map_pcl_debug", 1);
  ros::spin();

  return 0;
}
