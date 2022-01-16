#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
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
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point(1, 1, 1);
  cloud.points.push_back(point);
  cloud.header.frame_id = "/lidar";

  tf::Quaternion quaternion_mag;
  quaternion_mag.setRPY(0, 0, offset);
  tf::Transform trans;
  trans.setRotation(quaternion_mag);
  pcl_ros::transformPointCloud(cloud, cloud, trans);
  return cloud;
}

TEST_F(TestScanToPointCloud, ComparisonTest)
{
  double min_dist;
  node_handle.getParam("scan_to_pointcloud/min_dist", min_dist);
  ASSERT_TRUE(min_dist == 0.1);

  double neighbor_dist;
  node_handle.getParam("scan_to_pointcloud/neighbor_dist", neighbor_dist);
  ASSERT_TRUE(neighbor_dist == 0.2);

  double offset;
  node_handle.getParam("scan_to_pointcloud/offset", offset);
  ASSERT_TRUE(offset == 2.35619449019);

  MockSubscriber<pcl::PointCloud<pcl::PointXYZ>> mock_sub("/pc2");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

  auto cloudComparer = [](const pcl::PointCloud<pcl::PointXYZ>& p1, const pcl::PointCloud<pcl::PointXYZ>& p2) {
    double difference =
        (p1.points[0].x - p2.points[0].x) + (p1.points[0].y - p2.points[0].y) + (p1.points[0].z - p2.points[0].z);
    return (difference < .01);
  };

  pcl::PointCloud<pcl::PointXYZ> test_cloud;
  pcl::PointXYZ point(-1.4142136, 0, 1);
  test_cloud.points.push_back(point);
  test_cloud.header.frame_id = "/lidar";

  mock_pub.publish(createPointCloudMsg(offset));
  ASSERT_TRUE(mock_sub.spinUntilMessages());
  const pcl::PointCloud<pcl::PointXYZ>& response_cloud = mock_sub.front();
  ASSERT_TRUE(response_cloud.size() > 0);

  ASSERT_TRUE(cloudComparer(response_cloud, test_cloud));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_to_pointcloud");
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
