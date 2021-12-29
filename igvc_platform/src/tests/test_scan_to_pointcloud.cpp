#include <gtest/gtest.h>
#include <nodes/scan_to_pointcloud.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mocking_utils/mock_subscriber.h>

class TestScanToPointcloud : public testing::Test
{
public:
  TestScanToPointcloud() : mock_pub(node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/pc2", 1))
  {
  }

protected:
  ros::NodeHandle node_handle;
  ros::Publisher mock_pub;
};

pcl::Pointcloud<pcl::PointXYZ> createPointCloudMsg()
{
  pcl::Pointcloud<pcl::PointXYZ> pointcloud_msg();
  return pointcloud_msg;
}

TEST_F(TestScanToPointcloud, ComparisonTest)
{
  MockSubscriber<pcl::PointCloud<pcl::PointXYZ>> mock_sub("/scan");
  ASSERT_TRUE(mock_sub.waitForPublisher());
  ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));  // seems off

  pcl::PointCloud<pcl::PointXYZ> test();

  mock_pub.publish(createPointCloudMsg());
  ASSERT_TRUE(mock_sub.spinUntilMessages());

  ASSERT_EQ(mock_sub.messages().size(), 1LU);
  const igvc_messages::pcl::PointCloud<pcl::PointXYZ> > & response = mock_sub.front();

  ASSERT_EQ(response, test);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_scan_to_pointcloud");
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
