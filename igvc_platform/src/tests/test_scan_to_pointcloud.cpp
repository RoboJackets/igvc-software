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
  TestScanToPointCloud() : mock_pub(node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pc2", 1))
  {
  }

protected:
  ros::NodeHandle node_handle;
  ros::Publisher mock_pub;
};

// slight modification of lines 44-56 of scan_to_pointcloud.cpp
pcl::PointCloud<pcl::PointXYZ> createPointCloudMsg(double offset)
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
  return (*cloud_for_pub);
}

TEST_F(TestScanToPointCloud, ComparisonTest)
{
  double offset1{ M_PI / 2 };
  double offset2{ 5 * M_PI / 2 };

  MockSubscriber<pcl::PointCloud<pcl::PointXYZ>> mock_sub("/pc2");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  auto cloudComparer = [](const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2) {
    double difference =
        (p1.points[0].x - p2.points[0].x) + (p1.points[0].y - p2.points[0].y) + (p1.points[0].z - p2.points[0].z);
    return (difference < .1);
  };
  
  mock_pub.publish(createPointCloudMsg(offset1));
  ASSERT_TRUE(mock_sub.spinUntilMessages());
  const pcl::PointCloud<pcl::PointXYZ>& response1 = mock_sub.front();
  ASSERT_TRUE(response1.size() > 0);

  mock_pub.publish(createPointCloudMsg(offset2));
  ASSERT_TRUE(mock_sub.spinUntilMessages());
  const pcl::PointCloud<pcl::PointXYZ>& response2 = mock_sub.front();
  ASSERT_TRUE(response2.size() > 0);

  ASSERT_TRUE(cloudComparer(response1, response2));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_to_pointcloud");
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
