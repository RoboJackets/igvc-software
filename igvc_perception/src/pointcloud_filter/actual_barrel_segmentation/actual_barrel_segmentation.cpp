#include <ros/ros.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
std::string pub_topic = "";
std::string sub_topic = "";

double clusterTolerance;
int minClusterSize;
int maxClusterSize;

void clusteringCallBack(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pointcloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pointcloud);
    ec.extract (cluster_indices);

    pub.publish(cluster_indices);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "week5");

    ros::NodeHandle nh;

    ros::NodeHandle pnh{"~"};
    pnh.getParam("clusterTolerance", clusterTolerance);
    pnh.getParam("minClusterSize", minClusterSize);
    pnh.getParam("maxClusterSize", maxClusterSize);

    pub = nh.advertise<std::vector<pcl::PointIndices>>(pub_topic, 1);
    ros::Subscriber pointcloud_sub = nh.subscribe(sub_topic, 1, clusteringCallBack);

    ros::spin();
}