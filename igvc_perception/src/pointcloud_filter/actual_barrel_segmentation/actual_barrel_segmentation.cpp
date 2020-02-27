#include <ros/ros.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <igvc_msgs/barrels.h>
#include <exception>

ros::Publisher clusterCountPub;
ros::Publisher clusterVisPub;
ros::Publisher barrelInfoPub;
std::string subTopic = "/nonground";
std::string countPubTopic = "/countOutput";
std::string visPubTopic = "/vizOutput";
std::string barrelPubTopic = "/barrelPos";

double clusterTolerance;
int minClusterSize;
int maxClusterSize;
double cylinderMinRad;
double cylinderMaxRad;
double cylinderDistThres;
double cylinderNormalDistWeight;
bool showCyl;
bool showClus;
bool showInlier;

void getCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder,
                 pcl::PointIndices::Ptr inliers_cylinder)
{
  // Cylinder Fitting Init
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  // Cylinder Fitting Variables
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(cylinderNormalDistWeight);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(cylinderDistThres);
  seg.setRadiusLimits(cylinderMinRad, cylinderMaxRad);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
}

void clusteringCallBack(const sensor_msgs::PointCloud2ConstPtr& pointcloudMsg)
{
  // Converting Sensor message to pcl::PCLPointCloud2
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*pointcloudMsg, *cloud);
  pcl::PointCloud<pcl::PointXYZ>* cloudXYZ = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr(cloudXYZ);
  pcl::fromPCLPointCloud2(*cloud, *cloudXYZPtr);

  // Clustering Init
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloudXYZPtr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // Clustering
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minClusterSize);
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudXYZPtr);
  ec.extract(cluster_indices);

  // Publish the number of clusters
  std_msgs::Int32 clusterCount;
  clusterCount.data = cluster_indices.size();
  clusterCountPub.publish(clusterCount);

  // Visualization Message and Barrel Info Init
  visualization_msgs::MarkerArray clusters_vis;
  igvc_msgs::barrels barrelInfo;

  for (int clusterInd = 0; clusterInd < (int)cluster_indices.size(); clusterInd++)
  {
    // Visualization Init
    visualization_msgs::Marker clusterPoints;
    visualization_msgs::Marker inlierPoints;

    // PointClout Init
    pcl::PointCloud<pcl::PointXYZ> curr_cloud;

    // Cluster Storing
    pcl::PointCloud<pcl::PointXYZ>* currClus = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currCloudXYZPtr(currClus);

    // Barrel Points
    clusterPoints.header.frame_id = "/lidar";
    clusterPoints.header.stamp = ros::Time::now();
    clusterPoints.ns = "Barrel_" + std::to_string(clusterInd);
    clusterPoints.action = visualization_msgs::Marker::ADD;
    clusterPoints.pose.orientation.w = 1.0;
    clusterPoints.id = 0;
    clusterPoints.type = visualization_msgs::Marker::POINTS;
    clusterPoints.scale.x = 0.01;
    clusterPoints.scale.y = 0.01;

    clusterPoints.color.a = 1.0;
    clusterPoints.color.r = clusterInd * 0.25;
    clusterPoints.color.g = clusterInd * 0.25;
    clusterPoints.color.b = clusterInd * 0.25;
    clusterPoints.lifetime = ros::Duration(0.1);

    for (int ptInd : cluster_indices.at(clusterInd).indices)
    {
      pcl::PointXYZ curr = cloudXYZPtr->points[ptInd];

      geometry_msgs::Point p;
      p.x = curr.x;
      p.y = curr.y;
      p.z = curr.z;
      clusterPoints.points.push_back(p);

      currClus->push_back(curr);
    }
    // Variables to store cylinder fitting result
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

    // Get Cylinder Fitting from the current cluster
    getCylinder(currCloudXYZPtr, coefficients_cylinder, inliers_cylinder);
    if (coefficients_cylinder->values.size() == 0)
    {
      continue;
    }

    // Extract Cylinder Info
    // X = a1 + v1*t
    // Y = a2 + v2*t
    // Z = a3 + v3*t
    double a1 = coefficients_cylinder->values[0];
    double a2 = coefficients_cylinder->values[1];
    double a3 = coefficients_cylinder->values[2];
    double v1 = coefficients_cylinder->values[3];
    double v2 = coefficients_cylinder->values[4];
    double v3 = coefficients_cylinder->values[5];
    double r = coefficients_cylinder->values[6];

    // Solve t for Z = 0
    double t = -a3 / v3;

    // Cylinder Inlier Points
    inlierPoints.header.frame_id = "/lidar";
    inlierPoints.header.stamp = ros::Time::now();
    inlierPoints.ns = "CylPoints_" + std::to_string(clusterInd);
    inlierPoints.action = visualization_msgs::Marker::ADD;
    inlierPoints.pose.orientation.w = 1.0;
    inlierPoints.id = 0;
    inlierPoints.type = visualization_msgs::Marker::POINTS;
    inlierPoints.scale.x = 0.01;
    inlierPoints.scale.y = 0.01;
    inlierPoints.color.a = 1.0;
    inlierPoints.color.r = 1.0;
    inlierPoints.color.g = 0.0;
    inlierPoints.color.b = 0.0;
    inlierPoints.lifetime = ros::Duration(0.1);

    for (int ptInd : inliers_cylinder->indices)
    {
      pcl::PointXYZ curr = currCloudXYZPtr->points[ptInd];

      geometry_msgs::Point pt_real;
      pt_real.x = curr.x;
      pt_real.y = curr.y;
      pt_real.z = curr.z;
      inlierPoints.points.push_back(pt_real);
    }

    // Setting up the Cylinder Visualizer
    visualization_msgs::Marker cylinder;
    cylinder.header.frame_id = "/lidar";
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = "Cylinder_" + std::to_string(clusterInd);
    ;
    cylinder.id = 0;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.pose.position.x = a1 + v1 * t;
    cylinder.pose.position.y = a2 + v2 * t;
    cylinder.pose.position.z = 0;
    cylinder.pose.orientation.x = 0.0;
    cylinder.pose.orientation.y = 0.0;
    cylinder.pose.orientation.z = 0.0;
    cylinder.pose.orientation.w = 1.0;
    cylinder.scale.x = r;
    cylinder.scale.y = r;
    cylinder.scale.z = 1;
    cylinder.color.r = 0.0f;
    cylinder.color.g = 1.0f;
    cylinder.color.b = 0.0f;
    cylinder.color.a = 1.0;
    cylinder.lifetime = ros::Duration(0.1);

    // Put barrel info into a ros message
    geometry_msgs::Point currBarrel;
    currBarrel.x = a1 + v1 * t;
    currBarrel.y = a2 + v2 * t;
    currBarrel.z = 0;

    barrelInfo.barrels.push_back(currBarrel);

    // Adding clusterPoints to Maker Array
    if (showClus)
    {
      clusters_vis.markers.push_back(clusterPoints);
    }
    if (showCyl)
    {
      clusters_vis.markers.push_back(cylinder);
    }
    if (showInlier)
    {
      clusters_vis.markers.push_back(inlierPoints);
    }
  }
  clusterVisPub.publish(clusters_vis);
  barrelInfoPub.publish(barrelInfo);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actualBarrelSegmentation");

  ros::NodeHandle nh;

  ros::NodeHandle pnh{ "~" };
  pnh.getParam("clusterTolerance", clusterTolerance);
  pnh.getParam("minClusterSize", minClusterSize);
  pnh.getParam("maxClusterSize", maxClusterSize);
  pnh.getParam("cylinderMinRad", cylinderMinRad);
  pnh.getParam("cylinderMaxRad", cylinderMaxRad);
  pnh.getParam("cylinderDistThres", cylinderDistThres);
  pnh.getParam("cylinderNormalDistWeight", cylinderNormalDistWeight);
  pnh.getParam("showCyl", showCyl);
  pnh.getParam("showClus", showClus);
  pnh.getParam("showInlier", showInlier);

  clusterCountPub = nh.advertise<std_msgs::Int32>(countPubTopic, 1);
  clusterVisPub = nh.advertise<visualization_msgs::MarkerArray>(visPubTopic, 1);
  barrelInfoPub = nh.advertise<igvc_msgs::barrels>(barrelPubTopic, 1);
  ros::Subscriber pointcloud_sub = nh.subscribe(subTopic, 1, clusteringCallBack);

  ros::spin();
}