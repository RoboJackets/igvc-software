#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <parameter_assertions/assertions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include "velodyne_pointcloud/point_types.h"

/**
 * Reads from velodyne pointcloud and uses a pinhole camera model in order to correlate pixels to 3d points.
 * Publishes to a pointcloud topic, line_pointcloud, containing all 3d points that are part of a line.
 */

image_geometry::PinholeCameraModel g_cam_model;
tf::StampedTransform g_transform_lidar_to_cam;
tf::StampedTransform g_transform_cam_to_base;
tf::StampedTransform g_transform_lidar_to_base;
ros::Publisher g_line_pointcloud_pub;
ros::Subscriber g_image_sub;
ros::Subscriber g_lidar_pointcloud_sub;
sensor_msgs::CameraInfo::ConstPtr g_cam_info;
cv::Mat g_cv_img;
int g_resize_width;
int g_resize_height;
std::string g_line_topic;
std::string g_pointcloud_topic;
std::string g_lidar_topic;

/**
 * Finds the two nearest neighbors to the input point, constraining the nearest neighbors
 * to be from different lidar channels.
 *
 * @param point The point to find the nearest neighbors of
 * @param kdtree_list List of 16 kdtrees, one for each lidar channel
 * @param nearest1 The out parameter for the nearest pixel
 * @param nearest2 the out parameter for the second nearest pixel from a different channel
 */

void findNearestNeighbors(pcl::PointXY point, const std::vector<pcl::KdTreeFLANN<pcl::PointXY>> &kdtree_list,
                          pcl::PointXY &nearest1, pcl::PointXY &nearest2)
{
  float nearest_1_dist = std::numeric_limits<float>::max();
  float nearest_2_dist = std::numeric_limits<float>::max();
  for (const auto &kdtree : kdtree_list)
  {
    if (kdtree.getInputCloud() != nullptr)
    {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      if (pointNKNSquaredDistance[0] < nearest_1_dist)
      {
        nearest_1_dist = pointNKNSquaredDistance[0];
        nearest1 = kdtree.getInputCloud()->points[pointIdxNKNSearch[0]];
      }
      else if (pointNKNSquaredDistance[0] < nearest_2_dist)
      {
        nearest_2_dist = pointNKNSquaredDistance[0];
        nearest2 = kdtree.getInputCloud()->points[pointIdxNKNSearch[0]];
      }
    }
  }
}

/**
 * Interpolates the z value for a pixel specificed by point, given two nearest neighbors by
 * projecting the point onto the line between the two neighbors, and interpolating along that line.
 *
 * @param nearest1 1st nearest neighbor
 * @param nearest2 2nd nearest neighbor
 * @param point The point being interpolated
 * @param xyz_img The 2D array that correlates each pixel to an x,y,z point
 *
 * @return Interpolated z value for point
 */

double interpolateZ(pcl::PointXY nearest1, pcl::PointXY nearest2, cv::Point2d point,
                    const std::vector<std::vector<pcl::PointXYZ>> &xyz_img)
{
  cv::Point2d p0 = cv::Point2d(nearest1.x, nearest1.y);
  cv::Point2d p1 = cv::Point2d(nearest2.x, nearest2.y);
  cv::Matx21f a_to_b = cv::Matx21f(p1.x - p0.x, p1.y - p0.y);
  cv::Matx21f a_to_p = cv::Matx21f(point.x - p0.x, point.y - p0.y);
  cv::Matx21f unit_vect = a_to_b * (1 / cv::norm(a_to_b));
  cv::Matx21f projection = a_to_p.dot(unit_vect);
  cv::Point2d new_point = cv::Point2d(projection(0, 0) + p0.x, projection(1, 0) + p0.y);
  double dist = cv::norm(new_point - p0);
  double total_dist = cv::norm(p1 - p0);
  return xyz_img[p0.y][p0.x].z + dist * (xyz_img[p1.y][p1.x].z - xyz_img[p0.y][p0.x].z) / total_dist;
}

/**
 * Given a pixel, project it into 3D space onto the plane z=inter_z.
 *
 * @param inter_z Value of z for plane
 * @param point Pixel location in xyz_img
 * @param cv_point Pixel used to calculate projection onto plane
 * @param xyz_img The 2D array that correlates each pixel to an x,y,z point
 */

void projectPixelToPlane(double inter_z, pcl::PointXY point, cv::Point2d cv_point,
                         std::vector<std::vector<pcl::PointXYZ>> &xyz_img)
{
  cv::Point3d cv_proj_ray = g_cam_model.projectPixelTo3dRay(cv_point);
  tf::Vector3 proj_ray = tf::Vector3(cv_proj_ray.x, cv_proj_ray.y, cv_proj_ray.z);
  tf::Matrix3x3 rotate_matrix = g_transform_cam_to_base.getBasis();
  tf::Vector3 translation = g_transform_cam_to_base.getOrigin();
  tf::Vector3 trans_ray = rotate_matrix * proj_ray;
  double scale = (inter_z - translation.getZ()) / (trans_ray.getZ());
  xyz_img[point.y][point.x].x = trans_ray.getX() * scale + translation.getX();
  xyz_img[point.y][point.x].y = trans_ray.getY() * scale + translation.getY();
}

/**
 * Performs nearest neighbor interpolation for the z value of a specific pixel. Then uses
 * a pinhole camera model to get a ray for each pixel. The intersection of the ray and the plane
 * of the interpolated z is then used to figure out the x and y coordinates.
 *
 * @param xyz_img The 2D array that correlates each pixel to an x,y,z point
 * @param mapped A list of pixels that have already been mapped to x,y,z points
 * @param cloud_list A list of pointers to point clouds, one for each lidar channel
 */
void imageInterpolation(std::vector<std::vector<pcl::PointXYZ>> &xyz_img, std::vector<cv::Point2d> mapped,
                        std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> cloud_list)
{
  // KDtree for each lidar channel
  std::vector<pcl::KdTreeFLANN<pcl::PointXY>> kdtree_list(16);
  for (int i = 0; i < 16; i++)
  {
    if (!cloud_list[i]->empty())
    {
      kdtree_list[i].setInputCloud(cloud_list[i]);
    }
  }
// Interpolate z and compute x and y for each white pixel of image
#pragma omp parallel
#pragma omp for collapse(2)
  for (unsigned int r = 0; r < xyz_img.size(); r++)
  {
    for (unsigned int c = 0; c < xyz_img[0].size(); c++)
    {
      if (g_cv_img.at<uint8_t>(r, c) == 255 &&
          !(std::find(mapped.begin(), mapped.end(), cv::Point2d(c, r)) != mapped.end()))
      {
        pcl::PointXY point = { static_cast<float>(c), static_cast<float>(r) };
        pcl::PointXY nearest1 = point;
        pcl::PointXY nearest2 = point;
        findNearestNeighbors(point, kdtree_list, nearest1, nearest2);

        cv::Point2d cv_point = cv::Point2d(c, r);
        double inter_z = interpolateZ(nearest1, nearest2, cv_point, xyz_img);
        xyz_img[point.y][point.x].z = inter_z;

        projectPixelToPlane(inter_z, point, cv_point, xyz_img);
      }
    }
  }
}

/**
 * Checks if a given point is in the FOV of the camera. If so the points is transformed to base footprint and the
 * mapping of lidar point to pixel is added. The lidar point is then added to a pointcloud of all other points in its
 * ring in the FOV.
 *
 * @param point The given lidar point
 * @param mapped A list of points that have been mapped to pixels in the FOV
 * @param kdtree_cloud_list A list of pointclouds, one for each ring
 * @param xyz_img The 2D array that correlates each pixel to an x,y,z point
 */

void checkPointInFOV(velodyne_pointcloud::PointXYZIR point, std::vector<cv::Point2d> &mapped,
                     std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> &kdtree_cloud_list,
                     std::vector<std::vector<pcl::PointXYZ>> &xyz_img)
{
  tf::Vector3 lidar_point = tf::Vector3(point.x, point.y, point.z);
  tf::Matrix3x3 rotate_matrix = g_transform_cam_to_base.getBasis();
  tf::Vector3 translation = g_transform_lidar_to_cam.getOrigin();
  tf::Vector3 transformed_point = rotate_matrix * lidar_point + translation;
  cv::Point3d cv_point = cv::Point3d(transformed_point.getX(), transformed_point.getY(), transformed_point.getZ());
  cv::Point2d pixel = g_cam_model.project3dToPixel(cv_point);
  if (pixel.x >= 0 && pixel.x < g_cv_img.cols && pixel.y >= 0 && pixel.y < g_cv_img.rows)
  {
    mapped.emplace_back(pixel);
    int pixel_y = pixel.y;
    int pixel_x = pixel.x;
    tf::Matrix3x3 rotate_to_base = g_transform_lidar_to_base.getBasis();
    tf::Vector3 translate_to_base = g_transform_lidar_to_base.getOrigin();
    tf::Vector3 base_lidar_point = rotate_to_base * lidar_point + translate_to_base;
    xyz_img[pixel_y][pixel_x] =
        pcl::PointXYZ(base_lidar_point.getX(), base_lidar_point.getY(), base_lidar_point.getZ());
    pcl::PointXY pcl_point = { static_cast<float>(pixel_x), static_cast<float>(pixel_y) };
    kdtree_cloud_list[point.ring]->push_back(pcl_point);
  }
}

/**
 * Looks through the global cv_img and adds the lidar point corresponding to any pixels that are lines to a pointcloud.
 *
 * @param line_cloud The output pointcloud
 * @param xyz_img The 2D array that correlates each pixel to an x,y,z point
 */

void addLinesToPointcloud(pcl::PointCloud<pcl::PointXYZ> &line_cloud,
                          const std::vector<std::vector<pcl::PointXYZ>> &xyz_img)
{
  for (unsigned int r = 0; r < xyz_img.size(); r++)
  {
    std::vector<pcl::PointXYZ> row = xyz_img[r];
    for (unsigned int c = 0; c < row.size(); c++)
    {
      if (g_cv_img.at<uint8_t>(r, c) == 255)
      {
        pcl::PointXYZ point = row[c];
        line_cloud.push_back(point);
      }
    }
  }
}

/**
 * Get transforms needed for camera projections between the camera and lidar and to eventually make everything in base
 * footprint.
 */
void getTransforms()
{
  tf::TransformListener tf_listener;
  if (tf_listener.waitForTransform("/cam/center_optical", "/lidar", ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform("/cam/center_optical", "/lidar", ros::Time(0), g_transform_lidar_to_cam);
  }
  else
  {
    ROS_ERROR_STREAM("\n\nfailed to find lidar to camera transform\n\n");
  }
  if (tf_listener.waitForTransform("/base_footprint", "/cam/center_optical", ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform("/base_footprint", "/cam/center_optical", ros::Time(0), g_transform_cam_to_base);
  }
  else
  {
    ROS_ERROR_STREAM("\n\nfailed to find camera to base footprint transform\n\n");
  }
  if (tf_listener.waitForTransform("/base_footprint", "/lidar", ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform("/base_footprint", "/lidar", ros::Time(0), g_transform_lidar_to_base);
  }
  else
  {
    ROS_ERROR_STREAM("\n\nfailed to find lidar to base footprint transform\n\n");
  }
}
/**
 * Resize the pinhole camera model with the given resize_width and resize_height;
 */
void resizeCameraModel()
{
  sensor_msgs::CameraInfo new_g_cam_info(*g_cam_info);
  float waf = float(g_resize_width) / g_cam_info->width;
  float haf = float(g_resize_height) / g_cam_info->height;
  new_g_cam_info.height = g_resize_height;
  new_g_cam_info.width = g_resize_width;
  boost::array<double, 9> new_K = {
    g_cam_info->K[0] * waf, 0., g_cam_info->K[2] * waf, 0., g_cam_info->K[4] * haf, g_cam_info->K[5] * haf, 0., 0., 1.
  };
  boost::array<double, 12> new_P = { g_cam_info->P[0] * waf,
                                     0.,
                                     g_cam_info->P[2] * waf,
                                     0.,
                                     0.,
                                     g_cam_info->P[5] * haf,
                                     g_cam_info->P[6] * haf,
                                     0.,
                                     0.,
                                     0.,
                                     1.,
                                     0. };
  new_g_cam_info.K = new_K;
  new_g_cam_info.P = new_P;
  g_cam_model = image_geometry::PinholeCameraModel();
  g_cam_model.fromCameraInfo(new_g_cam_info);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat original_img = cv_bridge::toCvCopy(msg, "mono8")->image;
  cv::resize(original_img, g_cv_img, cv::Size(g_resize_width, g_resize_height), 0, 0);
}

void lidarPointcloudCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
{
  if (!g_cv_img.rows)
  {
    ROS_INFO_STREAM("Waiting for segmented image \n");
    return;
  }
  std::vector<cv::Point2d> mapped;
  std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> kdtree_cloud_list(16);
  for (int i = 0; i < 16; i++)
  {
    kdtree_cloud_list[i] = std::unique_ptr<pcl::PointCloud<pcl::PointXY>>(new pcl::PointCloud<pcl::PointXY>);
  }
  std::vector<std::vector<pcl::PointXYZ>> xyz_img(g_cv_img.rows, std::vector<pcl::PointXYZ>(g_cv_img.cols));
  pcl::PointCloud<pcl::PointXYZ> line_cloud;
  for (const auto &point : msg->points)
  {
    checkPointInFOV(point, mapped, kdtree_cloud_list, xyz_img);
  }
  imageInterpolation(xyz_img, mapped, kdtree_cloud_list);
  addLinesToPointcloud(line_cloud, xyz_img);
  line_cloud.header.frame_id = "base_footprint";
  g_line_pointcloud_pub.publish(line_cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projection");

  ros::NodeHandle n;
  ros::NodeHandle pNh("~");

  assertions::getParam(pNh, "resize_width", g_resize_width);
  assertions::getParam(pNh, "resize_height", g_resize_height);
  assertions::getParam(pNh, "line_topic", g_line_topic);
  assertions::getParam(pNh, "pointcloud_topic", g_pointcloud_topic);
  assertions::getParam(pNh, "lidar_topic", g_lidar_topic);

  g_cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("cam/center/camera_info", ros::Duration(5));
  if (g_cam_info.get() != nullptr)
  {
    resizeCameraModel();
    getTransforms();
  }
  else
  {
    ROS_ERROR_STREAM("\n\nfailed to find camera info\n\n");
  }
  g_image_sub = n.subscribe(g_line_topic, 1, imageCallback);
  g_lidar_pointcloud_sub = n.subscribe(g_lidar_topic, 1, lidarPointcloudCallback);
  g_line_pointcloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>>(g_pointcloud_topic, 1);
  ros::spin();
}
