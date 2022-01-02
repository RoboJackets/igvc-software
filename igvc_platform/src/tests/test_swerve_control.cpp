#include <gtest/gtest.h>
#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <mocking_utils/mock_subscriber.h>

class TestSwerveControl : public testing::Test
{
public:
  TestSwerveControl()
    : mock_motors_pub(handle.advertise<igvc_msgs::velocity_quad>("/motors", 1))
    , mock_joint_pub(handle.advertise<sensor_msgs::JointState>("/joint_states", 1))
  {
  }

protected:
  ros::NodeHandle handle;
  ros::Publisher mock_motors_pub;
  ros::Publisher mock_joint_pub;
};

igvc_msgs::velocity_quad createVelocityQuad(double fl_vel, double fr_vel, double bl_vel, double br_vel, double fl_swiv,
                                            double fr_swiv, double bl_swiv, double br_swiv)
{
  // contructing messages for motors
  igvc_msgs::velocity_quad msg;
  msg.fl_velocity = fl_vel;
  msg.fr_velocity = fr_vel;
  msg.bl_velocity = bl_vel;
  msg.br_velocity = br_vel;
  msg.fl_angle = fl_swiv;
  msg.fr_angle = fr_swiv;
  msg.bl_angle = bl_swiv;
  msg.br_angle = br_swiv;
  return msg;
}

TEST_F(TestSwerveControl, StopTest)
{
  MockSubscriber<std_msgs::Float64> mock_e_fl("/front_left_effort_controller/command");  // front_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_fr(
      "/front_right_effort_controller/command");                                         // front_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_bl("/back_left_effort_controller/command");   // back_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_br("/back_right_effort_controller/command");  // back_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fl("/front_left_swivel_controller/command");  // front_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fr(
      "/front_right_swivel_controller/command");                                         // front_right_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_bl("/back_left_swivel_controller/command");   // back_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_br("/back_right_swivel_controller/command");  // back_right_swivel_publisher

  ASSERT_TRUE(mock_e_fl.waitForPublisher());
  ASSERT_TRUE(mock_e_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_fr.waitForPublisher());
  ASSERT_TRUE(mock_e_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_bl.waitForPublisher());
  ASSERT_TRUE(mock_e_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_br.waitForPublisher());
  ASSERT_TRUE(mock_e_br.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fl.waitForPublisher());
  ASSERT_TRUE(mock_s_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fr.waitForPublisher());
  ASSERT_TRUE(mock_s_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_bl.waitForPublisher());
  ASSERT_TRUE(mock_s_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_br.waitForPublisher());
  ASSERT_TRUE(mock_s_br.waitForSubscriber(mock_motors_pub));

  const float stop = 0.0;
  mock_motors_pub.publish(createVelocityQuad(stop, stop, stop, stop, stop, stop, stop, stop));

  ASSERT_TRUE(mock_e_fl.spinUntilMessages());
  ASSERT_EQ(mock_e_fl.messages().size(), 1LU);
  const std_msgs::Float64& response1 = mock_e_fl.front();
  EXPECT_EQ(response1.data, stop);

  ASSERT_TRUE(mock_e_fr.spinUntilMessages());
  ASSERT_EQ(mock_e_fr.messages().size(), 1LU);
  const std_msgs::Float64& response2 = mock_e_fr.front();
  EXPECT_EQ(response2.data, stop);

  ASSERT_TRUE(mock_e_bl.spinUntilMessages());
  ASSERT_EQ(mock_e_bl.messages().size(), 1LU);
  const std_msgs::Float64& response3 = mock_e_bl.front();
  EXPECT_EQ(response3.data, stop);

  ASSERT_TRUE(mock_e_br.spinUntilMessages());
  ASSERT_EQ(mock_e_br.messages().size(), 1LU);
  const std_msgs::Float64& response4 = mock_e_br.front();
  EXPECT_EQ(response4.data, stop);

  ASSERT_TRUE(mock_s_fl.spinUntilMessages());
  ASSERT_EQ(mock_s_fl.messages().size(), 1LU);
  const std_msgs::Float64& response5 = mock_s_fl.front();
  EXPECT_EQ(response5.data, stop);

  ASSERT_TRUE(mock_s_fr.spinUntilMessages());
  ASSERT_EQ(mock_s_fr.messages().size(), 1LU);
  const std_msgs::Float64& response6 = mock_s_fr.front();
  EXPECT_EQ(response6.data, stop);

  ASSERT_TRUE(mock_s_bl.spinUntilMessages());
  ASSERT_EQ(mock_s_bl.messages().size(), 1LU);
  const std_msgs::Float64& response7 = mock_s_bl.front();
  EXPECT_EQ(response7.data, stop);

  ASSERT_TRUE(mock_s_br.spinUntilMessages());
  ASSERT_EQ(mock_s_br.messages().size(), 1LU);
  const std_msgs::Float64& response8 = mock_s_br.front();
  EXPECT_EQ(response8.data, stop);
}

TEST_F(TestSwerveControl, ForwardTest)
{
  MockSubscriber<std_msgs::Float64> mock_e_fl("/front_left_effort_controller/command");  // front_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_fr(
      "/front_right_effort_controller/command");                                         // front_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_bl("/back_left_effort_controller/command");   // back_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_br("/back_right_effort_controller/command");  // back_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fl("/front_left_swivel_controller/command");  // front_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fr(
      "/front_right_swivel_controller/command");                                         // front_right_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_bl("/back_left_swivel_controller/command");   // back_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_br("/back_right_swivel_controller/command");  // back_right_swivel_publisher

  ASSERT_TRUE(mock_e_fl.waitForPublisher());
  ASSERT_TRUE(mock_e_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_fr.waitForPublisher());
  ASSERT_TRUE(mock_e_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_bl.waitForPublisher());
  ASSERT_TRUE(mock_e_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_br.waitForPublisher());
  ASSERT_TRUE(mock_e_br.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fl.waitForPublisher());
  ASSERT_TRUE(mock_s_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fr.waitForPublisher());
  ASSERT_TRUE(mock_s_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_bl.waitForPublisher());
  ASSERT_TRUE(mock_s_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_br.waitForPublisher());
  ASSERT_TRUE(mock_s_br.waitForSubscriber(mock_motors_pub));

  const float forward = 1.0;
  const float stop = 0.0;
  mock_motors_pub.publish(createVelocityQuad(forward, forward, forward, forward, stop, stop, stop, stop));

  ASSERT_TRUE(mock_e_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fl.messages().size(), 5LU);
  const std_msgs::Float64& response1 = mock_e_fl.back();
  EXPECT_GT(response1.data, stop);

  ASSERT_TRUE(mock_e_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fr.messages().size(), 5LU);
  const std_msgs::Float64& response2 = mock_e_fr.back();
  EXPECT_GT(response2.data, stop);

  ASSERT_TRUE(mock_e_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_bl.messages().size(), 5LU);
  const std_msgs::Float64& response3 = mock_e_bl.back();
  EXPECT_GT(response3.data, stop);

  ASSERT_TRUE(mock_e_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_br.messages().size(), 5LU);
  const std_msgs::Float64& response4 = mock_e_br.back();
  EXPECT_GT(response4.data, stop);

  ASSERT_TRUE(mock_s_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fl.messages().size(), 5LU);
  const std_msgs::Float64& response5 = mock_s_fl.back();
  EXPECT_EQ(response5.data, stop);

  ASSERT_TRUE(mock_s_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fr.messages().size(), 5LU);
  const std_msgs::Float64& response6 = mock_s_fr.back();
  EXPECT_EQ(response6.data, stop);

  ASSERT_TRUE(mock_s_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_bl.messages().size(), 5LU);
  const std_msgs::Float64& response7 = mock_s_bl.back();
  EXPECT_EQ(response7.data, stop);

  ASSERT_TRUE(mock_s_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_br.messages().size(), 5LU);
  const std_msgs::Float64& response8 = mock_s_br.back();
  EXPECT_EQ(response8.data, stop);
}

TEST_F(TestSwerveControl, ReverseTest)
{
  MockSubscriber<std_msgs::Float64> mock_e_fl("/front_left_effort_controller/command");  // front_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_fr(
      "/front_right_effort_controller/command");                                         // front_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_bl("/back_left_effort_controller/command");   // back_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_br("/back_right_effort_controller/command");  // back_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fl("/front_left_swivel_controller/command");  // front_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fr(
      "/front_right_swivel_controller/command");                                         // front_right_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_bl("/back_left_swivel_controller/command");   // back_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_br("/back_right_swivel_controller/command");  // back_right_swivel_publisher

  ASSERT_TRUE(mock_e_fl.waitForPublisher());
  ASSERT_TRUE(mock_e_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_fr.waitForPublisher());
  ASSERT_TRUE(mock_e_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_bl.waitForPublisher());
  ASSERT_TRUE(mock_e_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_br.waitForPublisher());
  ASSERT_TRUE(mock_e_br.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fl.waitForPublisher());
  ASSERT_TRUE(mock_s_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fr.waitForPublisher());
  ASSERT_TRUE(mock_s_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_bl.waitForPublisher());
  ASSERT_TRUE(mock_s_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_br.waitForPublisher());
  ASSERT_TRUE(mock_s_br.waitForSubscriber(mock_motors_pub));

  const float backwards = -1.0;
  const float stop = 0.0;
  mock_motors_pub.publish(createVelocityQuad(backwards, backwards, backwards, backwards, stop, stop, stop, stop));

  ASSERT_TRUE(mock_e_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fl.messages().size(), 5LU);
  const std_msgs::Float64& response1 = mock_e_fl.back();
  EXPECT_LT(response1.data, stop);

  ASSERT_TRUE(mock_e_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fr.messages().size(), 5LU);
  const std_msgs::Float64& response2 = mock_e_fr.back();
  EXPECT_LT(response2.data, stop);

  ASSERT_TRUE(mock_e_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_bl.messages().size(), 5LU);
  const std_msgs::Float64& response3 = mock_e_bl.back();
  EXPECT_LT(response3.data, stop);

  ASSERT_TRUE(mock_e_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_br.messages().size(), 5LU);
  const std_msgs::Float64& response4 = mock_e_br.back();
  EXPECT_LT(response4.data, stop);

  ASSERT_TRUE(mock_s_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fl.messages().size(), 5LU);
  const std_msgs::Float64& response5 = mock_s_fl.back();
  EXPECT_EQ(response5.data, stop);

  ASSERT_TRUE(mock_s_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fr.messages().size(), 5LU);
  const std_msgs::Float64& response6 = mock_s_fr.back();
  EXPECT_EQ(response6.data, stop);

  ASSERT_TRUE(mock_s_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_bl.messages().size(), 5LU);
  const std_msgs::Float64& response7 = mock_s_bl.back();
  EXPECT_EQ(response7.data, stop);

  ASSERT_TRUE(mock_s_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_br.messages().size(), 5LU);
  const std_msgs::Float64& response8 = mock_s_br.back();
  EXPECT_EQ(response8.data, stop);
}

TEST_F(TestSwerveControl, ReverseTurnTest)
{
  MockSubscriber<std_msgs::Float64> mock_e_fl("/front_left_effort_controller/command");  // front_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_fr(
      "/front_right_effort_controller/command");                                         // front_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_bl("/back_left_effort_controller/command");   // back_left_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_e_br("/back_right_effort_controller/command");  // back_right_effort_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fl("/front_left_swivel_controller/command");  // front_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_fr(
      "/front_right_swivel_controller/command");                                         // front_right_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_bl("/back_left_swivel_controller/command");   // back_left_swivel_publisher
  MockSubscriber<std_msgs::Float64> mock_s_br("/back_right_swivel_controller/command");  // back_right_swivel_publisher

  ASSERT_TRUE(mock_e_fl.waitForPublisher());
  ASSERT_TRUE(mock_e_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_fr.waitForPublisher());
  ASSERT_TRUE(mock_e_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_bl.waitForPublisher());
  ASSERT_TRUE(mock_e_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_e_br.waitForPublisher());
  ASSERT_TRUE(mock_e_br.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fl.waitForPublisher());
  ASSERT_TRUE(mock_s_fl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_fr.waitForPublisher());
  ASSERT_TRUE(mock_s_fr.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_bl.waitForPublisher());
  ASSERT_TRUE(mock_s_bl.waitForSubscriber(mock_motors_pub));

  ASSERT_TRUE(mock_s_br.waitForPublisher());
  ASSERT_TRUE(mock_s_br.waitForSubscriber(mock_motors_pub));

  const float backwards = -1.0;
  const float stop = 0.0;
  const float turn90 = 1.57;
  mock_motors_pub.publish(
      createVelocityQuad(backwards, backwards, backwards, backwards, turn90, turn90, turn90, turn90));

  ASSERT_TRUE(mock_e_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fl.messages().size(), 5LU);
  const std_msgs::Float64& response1 = mock_e_fl.back();
  EXPECT_LT(response1.data, stop);

  ASSERT_TRUE(mock_e_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_fr.messages().size(), 5LU);
  const std_msgs::Float64& response2 = mock_e_fr.back();
  EXPECT_LT(response2.data, stop);

  ASSERT_TRUE(mock_e_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_bl.messages().size(), 5LU);
  const std_msgs::Float64& response3 = mock_e_bl.back();
  EXPECT_LT(response3.data, stop);

  ASSERT_TRUE(mock_e_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_e_br.messages().size(), 5LU);
  const std_msgs::Float64& response4 = mock_e_br.back();
  EXPECT_LT(response4.data, stop);

  ASSERT_TRUE(mock_s_fl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fl.messages().size(), 5LU);
  const std_msgs::Float64& response5 = mock_s_fl.back();
  EXPECT_EQ(response5.data, turn90);

  ASSERT_TRUE(mock_s_fr.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_fr.messages().size(), 5LU);
  const std_msgs::Float64& response6 = mock_s_fr.back();
  EXPECT_EQ(response6.data, turn90);

  ASSERT_TRUE(mock_s_bl.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_bl.messages().size(), 5LU);
  const std_msgs::Float64& response7 = mock_s_bl.back();
  EXPECT_EQ(response7.data, turn90);

  ASSERT_TRUE(mock_s_br.spinUntilMessages(ros::Duration{ 5 }, 5));
  ASSERT_EQ(mock_s_br.messages().size(), 5LU);
  const std_msgs::Float64& response8 = mock_s_br.back();
  EXPECT_EQ(response8.data, turn90);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_swerve_control");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}