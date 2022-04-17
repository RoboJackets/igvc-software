#include <gtest/gtest.h>
#include <mocking_utils/mock_subscriber.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <robot_localization/navsat_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mutex>

class TestGroundTruth: public testing::Test
{
public: 
	TestGroundTruth(): mock_pub(nh.advertise<nav_msgs::Odometry>("/ground_truth/state_raw", 1))
	{
	}


protected:
  ros::NodeHandle nh;
  ros::Publisher mock_pub;
};


nav_msgs::Odometry createOdomMsg(double initial_x, double initial_y, double time = 0.0){
	nav_msgs::Odometry odomMsg;
	odomMsg.pose.pose.position.x = initial_x;
	odomMsg.pose.pose.position.y = initial_y;
	odomMsg.child_frame_id = "base_footprint";
    odomMsg.header.frame_id = "odom";
	odomMsg.header.stamp = ros::Time(time);

	ROS_INFO_STREAM("Timestamp: " << odomMsg.header.stamp.toSec());
	return odomMsg;
};


TEST_F(TestGroundTruth, InitializationTest){

	//asserting all the parameters
	std::string ground_truth_sub_topic;
	nh.param("ground_truth_republisher/ground_truth_sub_topic", ground_truth_sub_topic, std::string("/ground_truth/state_raw"));
	ASSERT_TRUE(ground_truth_sub_topic == "/ground_truth/state_raw");

	std::string ground_truth_pub_topic;
	nh.param("ground_truth_republisher/ground_truth_pub_topic", ground_truth_pub_topic, std::string("/ground_truth"));
	ASSERT_TRUE(ground_truth_pub_topic == "/ground_truth");

	double longitude;
	nh.param("ground_truth_republisher/longitude", longitude, -84.405001);
	ASSERT_TRUE(longitude == -84.405001);

	double latitude;
	nh.param("ground_truth_republisher/latitude", latitude, 33.774497);
	ASSERT_TRUE(latitude== 33.774497);

	MockSubscriber<nav_msgs::Odometry> mock_sub("/ground_truth");
	ASSERT_TRUE(mock_sub.waitForPublisher());
	ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

	mock_pub.publish(createOdomMsg(1,2));

	ASSERT_TRUE(mock_sub.spinUntilMessages());

  	ASSERT_EQ(mock_sub.messages().size(), 1LU);

	nav_msgs::Odometry odomMsg = mock_sub.back();
	
	ASSERT_EQ(odomMsg.pose.pose.position.x, 0);
	ASSERT_EQ(odomMsg.pose.pose.position.y, 0);
}

TEST_F(TestGroundTruth, UpdateTest){

	MockSubscriber<nav_msgs::Odometry> mock_sub("/ground_truth");
	ASSERT_TRUE(mock_sub.waitForPublisher());
	ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub));

	//should treat (1,2) as (0, 0)
	mock_pub.publish(createOdomMsg(1, 2)); // dummy initial position
	ASSERT_TRUE(mock_sub.spinUntilMessages());

	mock_pub.publish(createOdomMsg(4, 6, 1.0)); // dummy second position 
	ASSERT_TRUE(mock_sub.spinUntilMessages(ros::Duration{5}, 2));

  	ASSERT_EQ(mock_sub.messages().size(), 2LU);

	nav_msgs::Odometry response_msg = mock_sub.back();

	ASSERT_EQ(response_msg.pose.pose.position.x, 3);
	ASSERT_EQ(response_msg.pose.pose.position.y, 4);
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_ground_truth");
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}

