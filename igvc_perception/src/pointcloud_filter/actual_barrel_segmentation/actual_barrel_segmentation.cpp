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

ros::Publisher clusterCountPub;
ros::Publisher clusterVisPub;
std::string count_pub_topic = "/countOutput";
std::string vis_pub_topic = "/vizOutput";
std::string sub_topic = "/nonground";

double clusterTolerance;
int minClusterSize;
int maxClusterSize;

void clusteringCallBack(const sensor_msgs::PointCloud2ConstPtr& pointcloudMsg) {
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*pointcloudMsg, *cloud);

    pcl::PointCloud<pcl::PointXYZ> *cloudXYZ = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr (cloudXYZ);

    pcl::fromPCLPointCloud2(*cloud, *cloudXYZPtr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudXYZPtr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudXYZPtr);
    ec.extract (cluster_indices);

    std_msgs::Int32 clusterCount;
    clusterCount.data = cluster_indices.size();

    clusterCountPub.publish(clusterCount);

    visualization_msgs::MarkerArray clusters_vis;

    for(int clusterInd = 0; clusterInd < (int)cluster_indices.size(); clusterInd++) {
        //Visualization Init
        visualization_msgs::Marker points;

        //PointClout Init
        pcl::PointCloud<pcl::PointXYZ> curr_cloud;

        points.header.frame_id = "/lidar";
        points.header.stamp = ros::Time::now();
        points.ns = "barrel_segmentation" + std::to_string(clusterInd);
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.01;
        points.scale.y = 0.01;

        points.color.a = 1.0;
        points.color.r = clusterInd*0.25;
        points.color.g = clusterInd*0.25;
        points.color.b = clusterInd*0.25;

        for (int ptInd : cluster_indices.at(clusterInd).indices)
        {
            pcl::PointXYZ curr = cloudXYZPtr->points[ptInd];
            pcl::PointCloud<pcl::PointXYZ> *currClus = new pcl::PointCloud<pcl::PointXYZ>;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr (currClus);
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

            geometry_msgs::Point p;
            p.x = curr.x;
            p.y = curr.y;
            p.z = curr.z;
            points.points.push_back(p);

            currClus->push_back(curr);
        }
        clusters_vis.markers.push_back(points);
    }

    clusterVisPub.publish(clusters_vis);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "actualBarrelSegmentation");

    ros::NodeHandle nh;

    ros::NodeHandle pnh{"~"};
    pnh.getParam("clusterTolerance", clusterTolerance);
    pnh.getParam("minClusterSize", minClusterSize);
    pnh.getParam("maxClusterSize", maxClusterSize);

    clusterCountPub = nh.advertise<std_msgs::Int32>(count_pub_topic, 1);
    clusterVisPub = nh.advertise<visualization_msgs::MarkerArray>(vis_pub_topic, 1);
    ros::Subscriber pointcloud_sub = nh.subscribe(sub_topic, 1, clusteringCallBack);

    ros::spin();
}