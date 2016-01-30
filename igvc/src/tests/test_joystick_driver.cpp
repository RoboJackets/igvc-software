//
// Created by matt on 1/29/16.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <igvc_msgs/velocity_pair.h>
#include <sensor_msgs/Joy.h>

class TestJoystickDriver : public testing::Test {

public:
    TestJoystickDriver()
      : handle(),
        mock_joy_pub(handle.advertise<sensor_msgs::Joy>("/joy", 1)),
        motor_sub(handle.subscribe("/motors", 1, &TestJoystickDriver::motorsCallback, this))
    {
    }

    void motorsCallback(const igvc_msgs::velocity_pair::ConstPtr& msg) {
      received = true;
    }

protected:
    virtual void SetUp() {
      received = false;
      while(!IsNodeReady()) {
        ros::spinOnce();
      }
    }

    virtual void TearDown() {

    }

    bool IsNodeReady() {
      return (mock_joy_pub.getNumSubscribers() > 0) && (motor_sub.getNumPublishers() > 0);
    }

    ros::NodeHandle handle;
    ros::Publisher mock_joy_pub;
    ros::Subscriber motor_sub;
    bool received;

};

TEST_F(TestJoystickDriver, TestJoystickDriver) {
  ros::Rate rate(1);

  sensor_msgs::Joy joy_msg;
  joy_msg.axes = {0, 1.0, 0, 1.0};
  joy_msg.buttons = {0, 0, 0, 0};
  mock_joy_pub.publish(joy_msg);

  rate.sleep(); // Give message time to send
  ros::spinOnce();

  EXPECT_TRUE(received);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_joystick_driver");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}