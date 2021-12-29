#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mocking_utils/mock_subscriber.h>

class TestScanToPointCloud : public testing::Test
{
public:
  TestScanToPointCloud()
    : mock_pub(node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pc2", 1))
    , mock_sub(node_handle.subscribe("/pc2", 1, &TestScanToPointCloud::sub_callback, this))
  {
  }

  void sub_callback(const pcl::PointCloud<pcl::PointXYZ>)  // best practice?
  {
  }

protected:
  ros::NodeHandle node_handle;
  ros::Publisher mock_pub;
  ros::Subscriber mock_sub;
};

// slight modification of lines 44-56 of scan_to_pointcloud.cpp
pcl::PointCloud<pcl::PointXYZ>::ConstPtr createPointCloudMsg(double offset)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "/lidar";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_pub(new pcl::PointCloud<pcl::PointXYZ>());
  fromROSMsg(cloud, *cloud_for_pub);
  tf::Quaternion quaternion_mag;
  quaternion_mag.setRPY(0, 0, offset);
  tf::Transform trans;
  trans.setRotation(quaternion_mag);
  pcl_ros::transformPointCloud(*cloud_for_pub, *cloud_for_pub, trans);
  return cloud_for_pub;
}

TEST_F(TestScanToPointCloud, ComparisonTest)
{
  double offset1{ M_PI / 2 };
  double offset2{ 5 * M_PI / 2 };

  auto cloudComparer = [](pcl::PointCloud<pcl::PointXYZ>::ConstPtr p1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr p2) {
    return (p1->points.at(0).x - p2->points.at(0).x) < .1;
  };

  mock_pub.publish(createPointCloudMsg(offset1));
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr response1 =
      ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>(mock_sub.getTopic(), ros::Duration(5));
  ASSERT_TRUE(response1 != nullptr);

  mock_pub.publish(createPointCloudMsg(offset2));
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr response2 =
      ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>(mock_sub.getTopic(), ros::Duration(5));
  ASSERT_TRUE(response2 != nullptr);

  bool result = cloudComparer(response1, response2);
  ASSERT_TRUE(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_to_pointcloud");
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
