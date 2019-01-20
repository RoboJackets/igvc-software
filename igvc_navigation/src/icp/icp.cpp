#include "icp.h"

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>

int main(int argc, char **argv) {
  ros::init(argc, argv, "icp");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  ICP icp;
  ros::Subscriber pcl_sub = nh.subscribe("/nonground_pcl", 1, &ICP::pc_callback, &icp);

  icp.scanmatch_pub = nh.advertise<igvc_msgs::map>("/map", 1);

  ros::spin();
}

void ICP::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
  if (m_last_cloud != nullptr)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(m_last_cloud);
    icp.setInputTarget(pc);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    ROS_INFO_STREAM("Score: " << icp.getFitnessScore());

    tf::Transform trans;
    Eigen::Affine3d affine;
    affine.matrix() = icp.getFinalTransformation().cast<double>();
    tf::transformEigenToTF(affine, trans);

    ros::Time messageTimeStamp;
    pcl_conversions::fromPCL(pc->header.stamp, messageTimeStamp);

//    tf::StampedTransform tf_trans;
//    tf_trans.setBasis(trans.getBasis());
//    tf_trans.stamp_ = messageTimeStamp;
//    tf_trans.child_frame_id_ = "base_footprint";
//    tf_trans.frame_id_ = "scanmatch";

    m_br.sendTransform(tf::StampedTransform(trans, messageTimeStamp, "base_footprint", "scanmatch"));
  }
  m_last_cloud = std::move(pc);
}

