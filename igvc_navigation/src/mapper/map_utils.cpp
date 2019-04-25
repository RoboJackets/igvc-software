#include <unordered_set>

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "map_utils.h"

namespace MapUtils
{
inline int discretize(radians angle, double angular_resolution)
{
  double coeff = 1 / angular_resolution;
  return static_cast<int>(std::round(coeff * angle));
}

void filterPointsBehind(const pcl::PointCloud<pcl::PointXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>& filtered_pc,
                        BehindFilterOptions options)
{
  static double end_angle = -M_PI + options.angle / 2;
  static double start_angle = M_PI - options.angle / 2;
  static double squared_distance = options.distance * options.distance;
  // Iterate over pointcloud, insert discretized angles into set
  for (auto i : pc)
  {
    double angle = atan2(i.y, i.x);
    if ((-M_PI <= angle && angle < end_angle) || (start_angle < angle && angle <= M_PI))
    {
      if (i.x * i.x + i.y * i.y > squared_distance)
      {
        filtered_pc.points.emplace_back(i);
      }
    }
    else
    {
      filtered_pc.points.emplace_back(i);
    }
  }
}

std::optional<GroundPlane> filterGroundPlane(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                             GroundFilterOptions options)
{
  ground.header = raw_pc.header;
  nonground.header = raw_pc.header;

  std::optional<GroundPlane> ground_plane = ransacFilter(raw_pc, ground, nonground, options.ransac_options);
  if (ground_plane)
  {
    return ground_plane;
  }
  else
  {
    fallbackFilter(raw_pc, ground, nonground, options.fallback_options);
    return std::nullopt;
  }
}

void projectTo2D(PointCloud& projected_pc)
{
  for (auto& point : projected_pc.points)
  {
    point.z = 0;
  }
}

std::optional<GroundPlane> ransacFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground,
                                        const RANSACOptions& options)
{
  // Plane detection for ground removal
  pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);

  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(options.iterations);
  seg.setDistanceThreshold(options.distance_threshold);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(options.eps_angle);

  PointCloud::Ptr cloud_filtered = raw_pc.makeShared();

  // Create filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    return std::nullopt;
  }
  else
  {
    ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
              cloud_filtered->size(), coefficients->values.at(0), coefficients->values.at(1),
              coefficients->values.at(2), coefficients->values.at(3));
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(ground);

    // remove ground points from full pointcloud
    if (inliers->indices.size() != cloud_filtered->size())
    {
      extract.setNegative(true);
      PointCloud out;
      extract.filter(out);
      nonground += out;
      *cloud_filtered = out;
    }

    return GroundPlane{ coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2),
                        coefficients->values.at(3) };
  }
}

void fallbackFilter(const PointCloud& raw_pc, PointCloud& ground, PointCloud& nonground, const FallbackOptions& options)
{
  pcl::PassThrough<pcl::PointXYZ> fallback;
  fallback.setFilterFieldName("z");
  fallback.setFilterLimits(-options.plane_distance, options.plane_distance);
  fallback.filter(ground);

  fallback.setFilterLimitsNegative(true);
  fallback.filter(nonground);
}

void blur(cv::Mat& blurred_map, double kernel_size)
{
  cv::Mat original = blurred_map.clone();
  cv::blur(blurred_map, blurred_map, cv::Size(kernel_size, kernel_size));
  cv::max(original, blurred_map, blurred_map);
}

void getEmptyPoints(const pcl::PointCloud<pcl::PointXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>& empty_pc,
                    double angular_resolution, EmptyFilterOptions options)
{
  // Iterate over pointcloud, insert discretized angles into set if within max range
  std::unordered_set<int> discretized_angles{};
  for (auto i : pc)
  {
    if (withinRange(i, options.max_range))
    {
      double angle = atan2(i.y, i.x);
      discretized_angles.emplace(MapUtils::discretize(angle, angular_resolution));
    }
  }

  // For each angle, if it's not in the set (empty), put it into a pointcloud.
  // From Robot's frame. Need to rotate angle to world frame
  for (int i = discretize(options.start_angle, angular_resolution);
       i < discretize(options.end_angle, angular_resolution); i++)
  {
    if (discretized_angles.find(i) == discretized_angles.end())
    {
      double angle = i * angular_resolution;
      pcl::PointXYZ point{ static_cast<float>(options.miss_cast_distance * cos(angle)),
                           static_cast<float>(options.miss_cast_distance * sin(angle)), 0 };
      empty_pc.points.emplace_back(point);
    }
  }
}

void removeOccupiedFromImage(cv::Mat& image, const PointCloud& last_scan,
                             const image_geometry::PinholeCameraModel& camera_model,
                             const tf::Transform& camera_to_odom, const RemoveOccupiedOptions& options)
{
  tf::Transform odom_to_camera = camera_to_odom.inverse();
  cv::Rect image_rect = cv::Rect{ 0, 0, image.size().width, image.size().height };

  cv::Mat projected_image(image.size().height, image.size().width, image.type(), static_cast<uchar>(0));

  std::vector<cv::Point2d> projected_points;
  projected_points.reserve(100);

  // Transform pointcloud to image frame, add if inside image
  for (const auto& point : last_scan.points)
  {
    tf::Vector3 transformed = odom_to_camera * tf::Vector3{ point.x, point.y, 0 };
    cv::Point3d cv_point{ transformed.x(), transformed.y(), transformed.z() };
    cv::Point2d projected_point = camera_model.project3dToPixel(cv_point);
    if (projected_point.inside(image_rect))
    {
      projected_points.emplace_back(projected_point);
      for (int i = 0; i < 10; i++)
      {
        tf::Vector3 transformed_heights = odom_to_camera * tf::Vector3{ point.x, point.y, i * 0.1 };
        cv::Point3d cv_point_heights{ transformed_heights.x(), transformed_heights.y(), transformed_heights.z() };
        cv::Point2d projected_point_heights = camera_model.project3dToPixel(cv_point_heights);
        if (projected_point_heights.inside(image_rect))
        {
          projected_points.emplace_back(projected_point_heights);
        }
      }
    }
  }

  // Set all pixels that are lidar points white so it isn't free space
  for (const auto& cv_point : projected_points)
  {
    projected_image.at<uchar>(static_cast<int>(std::round(cv_point.y)), static_cast<int>(std::round(cv_point.x))) =
        static_cast<uchar>(255);
  }

  // Do closing to make join up the dots
  cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * options.kernel_size + 1, 2 * options.kernel_size + 1));
  cv::morphologyEx(projected_image, projected_image, cv::MORPH_CLOSE, kernel);
  cv::max(projected_image, image, image);
}

void projectToPlane(PointCloud& projected_pc, const GroundPlane& ground_plane, const cv::Mat& image,
                    const image_geometry::PinholeCameraModel& model, const tf::Transform& camera_to_world)
{
  int nRows = image.rows;
  int nCols = image.cols;

  int i, j;
  const uchar* p;
  for (i = 0; i < nRows; ++i)
  {
    p = image.ptr<uchar>(i);
    for (j = 0; j < nCols; ++j)
    {
      // If it's a black pixel => free space, then project
      if (p[j] == 0)
      {
        cv::Point2d pixel_point(j, i);

        cv::Point3d ray = model.projectPixelTo3dRay(pixel_point);
        tf::Point reoriented_ray{ ray.x, ray.y, ray.z };  // cv::Point3d defined with z forward, x right, y down
        tf::Point transformed_ray = camera_to_world.getBasis() * reoriented_ray;  // Transform ray to odom frame
        double a = ground_plane.a;
        double b = ground_plane.b;
        double c = ground_plane.c;
        double d = ground_plane.d;

        // [a b c]^T dot (P0 - (camera + ray*t)) = 0, solve for t
        double t = ((tf::Vector3{ 0, 0, d / c } - camera_to_world.getOrigin()).dot(tf::Vector3{ a, b, c })) /
                   (transformed_ray.dot(tf::Vector3{ a, b, c }));

        // projected_point = camera + ray*t
        tf::Point projected_point = camera_to_world.getOrigin() + transformed_ray * t;
        projected_pc.points.emplace_back(pcl::PointXYZ(static_cast<float>(projected_point.x()),
                                                       static_cast<float>(projected_point.y()),
                                                       static_cast<float>(projected_point.z())));
      }
    }
  }
}

sensor_msgs::CameraInfoConstPtr scaleCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, double width,
                                                double height)
{
  sensor_msgs::CameraInfoPtr changed_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(*camera_info);
  changed_camera_info->D = camera_info->D;
  changed_camera_info->distortion_model = camera_info->distortion_model;
  changed_camera_info->R = camera_info->R;
  changed_camera_info->roi = camera_info->roi;
  changed_camera_info->binning_x = camera_info->binning_x;
  changed_camera_info->binning_y = camera_info->binning_y;

  double waf = static_cast<double>(width) / static_cast<double>(camera_info->width);
  double haf = static_cast<double>(height) / static_cast<double>(camera_info->height);

  changed_camera_info->width = static_cast<unsigned int>(width);
  changed_camera_info->height = static_cast<unsigned int>(height);

  changed_camera_info->K = { { camera_info->K[0] * waf, 0, camera_info->K[2] * waf, 0, camera_info->K[4] * haf,
                               camera_info->K[5] * haf, 0, 0, 1 } };
  changed_camera_info->P = { { camera_info->P[0] * waf, 0, camera_info->P[2] * waf, 0, 0, camera_info->P[5] * haf,
                               camera_info->P[6] * haf, 0, 0, 0, 1, 0 } };
  return changed_camera_info;
}

void debugPublishPointCloud(const ros::Publisher& publisher, pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                            const uint64 stamp, std::string&& frame, bool debug)
{
  if (debug)
  {
    pointcloud.header.stamp = stamp;
    pointcloud.header.frame_id = frame;
    publisher.publish(pointcloud);
  }
}
void debugPublishImage(const ros::Publisher& publisher, const cv::Mat& image, const ros::Time stamp, bool debug)
{
  if (debug)
  {
    cv_bridge::CvImage bridge_image;
    bridge_image.image = image;
    sensor_msgs::ImagePtr out = bridge_image.toImageMsg();
    out->encoding = "mono8";
    out->header.stamp = stamp;
    publisher.publish(out);
  }
}
}  // namespace MapUtils
