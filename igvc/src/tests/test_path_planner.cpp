#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/mat.hpp>

class graphSearchTest : public ::testing::Test
{
public:
  graphSearchTest()
  {
    // initialization code here
  }

  void SetUp()
  {
    // code here will execute just before the test ensues
  }

  void TearDown()
  {
    // code here will be called just after the test completes
    // ok to through exceptions from here if need be
  }

  ~graphSearchTest()
  {
    // cleanup any pending stuff, but no exceptions allowed
  }

  // put in any custom data members that you need
};

TEST_F(graphSearchTest, UnitTest1)
{
  EXPECT_EQ(18.0, 18.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
