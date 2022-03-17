#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <mocking_utils/mock_subscriber.h>
#include <parameter_assertions/assertions.h>

class TestSegmentedCamera : public testing::Test
{
public:
  TestSegmentedCamera() : pNH{ "~" }
  {
    // cameras to obtain images from
    assertions::getParam(nh, "segmented_camera/camera_names", camera_names);

    assertions::getParam(nh, "segmented_camera/output_width", output_width);
    assertions::getParam(nh, "segmented_camera/output_height", output_height);

    assertions::getParam(nh, "segmented_camera/segmented_publisher_path", seg_cam_pub_path);
    assertions::getParam(nh, "segmented_camera/segmented_subscriber_path", seg_cam_sub_path);

    // mock_pub will be a vector of [camera_names.size()] publishers
    for (size_t i = 0; i < camera_names.size(); i++)
    {
      auto camera_name = camera_names[i];

      // cam_center/raw/info
      std::string semantic_info_topic = camera_name + seg_cam_sub_path;

      // We publish to each camera's topic
      mock_pub.push_back(nh.advertise<sensor_msgs::CameraInfo>(semantic_info_topic, 1));
    }
  }

protected:
  // the private node handle will let us retrieve desired launch file params later
  ros::NodeHandle pNH;
  ros::NodeHandle nh;
  std::vector<std::string> camera_names;
  std::string seg_cam_pub_path, seg_cam_sub_path;
  // fake publisher we can use for testing
  std::vector<ros::Publisher> mock_pub;

  double output_width, output_height;
};

// make a fake CameraInfo message to transform
sensor_msgs::CameraInfo createCameraInfoMsg(double height_factor = 1.0, double width_factor = 1.0, double height = 1920,
                                            double width = 1080)
{
  sensor_msgs::CameraInfo camera_info;
  // all the properties that segmented_camera_info_publisher's ScaleCameraInfo() changes
  camera_info.height = height * height_factor;
  camera_info.width = width * width_factor;
  // these are probably not reasonable values, but they represent 3x3 matrices and that's what matters
  camera_info.K = { 1000, 0, 200, 3000, 120, 490, 0, 0, 1 };
  camera_info.P = { 480, 290, 1, 600, 1000, 0, 0, 120, 0, 0, 0, 0 };

  return camera_info;
}

// fake camera info message - hard coded transform of createCameraInfoMsg()
sensor_msgs::CameraInfo cameraInfoTransformedMsg(double output_width, double output_height, double height_factor = 1.0,
                                                 double width_factor = 1.0)
{
  sensor_msgs::CameraInfo camera_info = createCameraInfoMsg(height_factor, width_factor); 

  double w_ratio = static_cast<double>(output_width) / static_cast<double>(camera_info.width);
  double h_ratio = static_cast<double>(output_height) / static_cast<double>(camera_info.height);

  camera_info.width = static_cast<unsigned int>(output_width);
  camera_info.height = static_cast<unsigned int>(output_height);

  camera_info.K = { camera_info.K[0] * w_ratio,
                    0,
                    camera_info.K[2] * w_ratio,
                    0,
                    camera_info.K[4] * h_ratio,
                    camera_info.K[5] * h_ratio,
                    0,
                    0,
                    1 };
  camera_info.P = { camera_info.P[0] * w_ratio,
                    0,
                    camera_info.P[2] * w_ratio,
                    0,
                    0,
                    camera_info.P[5] * h_ratio,
                    camera_info.P[6] * h_ratio,
                    0,
                    0,
                    0,
                    1,
                    0 };

  return camera_info;
}

TEST_F(TestSegmentedCamera, NormalScalingTest)
{
  // publish fake camera info msgs to seg_cam's topics
  for (size_t i = 0; i < camera_names.size(); i++)
  {
    MockSubscriber<sensor_msgs::CameraInfo> mock_sub(camera_names[i] + seg_cam_pub_path);
    ASSERT_TRUE(mock_sub.waitForPublisher());
    ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub[i]));

    mock_pub[i].publish(createCameraInfoMsg());
    ASSERT_TRUE(mock_sub.spinUntilMessages());

    sensor_msgs::CameraInfo correctResponse = cameraInfoTransformedMsg(output_width, output_height);
    sensor_msgs::CameraInfo response = mock_sub.front();
    EXPECT_NEAR(correctResponse.width, response.width, 1E-100);
    EXPECT_NEAR(correctResponse.height, response.height, 1E-100);

    for (int i = 0; i < 9; i++)
    {
      EXPECT_NEAR(correctResponse.K[i], response.K[i], 1E-100);
    }
    for (int i = 0; i < 12; i++)
    {
      EXPECT_NEAR(correctResponse.P[i], response.P[i], 1E-100);
    }
  }
}

TEST_F(TestSegmentedCamera, OverScalingTest)  // scale to larger than original
{
  // publish fake camera info msgs to seg_cam's topics
  for (size_t i = 0; i < camera_names.size(); i++)
  {
    MockSubscriber<sensor_msgs::CameraInfo> mock_sub(camera_names[i] + seg_cam_pub_path);
    ASSERT_TRUE(mock_sub.waitForPublisher());
    ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub[i]));

    const double scale_factor = 10;

    mock_pub[i].publish(createCameraInfoMsg(scale_factor, scale_factor));
    ASSERT_TRUE(mock_sub.spinUntilMessages());

    sensor_msgs::CameraInfo correctResponse =
        cameraInfoTransformedMsg(output_width, output_height, scale_factor, scale_factor);
    sensor_msgs::CameraInfo response = mock_sub.front(); 
    EXPECT_NEAR(correctResponse.width, response.width, 1E-100);
    EXPECT_NEAR(correctResponse.height, response.height, 1E-100);

    for (int i = 0; i < 9; i++)
    {
      EXPECT_NEAR(correctResponse.K[i], response.K[i], 1E-100);
    }
    for (int i = 0; i < 12; i++)
    {
      EXPECT_NEAR(correctResponse.P[i], response.P[i], 1E-100);
    }
  }
}

// input all zeros
TEST_F(TestSegmentedCamera, ZeroTest)
{
  // publish fake camera info msgs to seg_cam's topics
  for (size_t i = 0; i < camera_names.size(); i++)
  {
    MockSubscriber<sensor_msgs::CameraInfo> mock_sub(camera_names[i] + seg_cam_pub_path);
    ASSERT_TRUE(mock_sub.waitForPublisher());
    ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub[i]));
    mock_pub[i].publish(createCameraInfoMsg(1.0, 1.0, 0, 0)); 

    // Expect no message publish since invalid inputs
    ASSERT_FALSE(mock_sub.spinUntilMessages());
  }
}

TEST_F(TestSegmentedCamera, UnevenScaling)  // uneven scaling
{
  // publish fake camera info msgs to seg_cam's topics
  for (size_t i = 0; i < camera_names.size(); i++)
  {
    MockSubscriber<sensor_msgs::CameraInfo> mock_sub(camera_names[i] + seg_cam_pub_path);
    ASSERT_TRUE(mock_sub.waitForPublisher());
    ASSERT_TRUE(mock_sub.waitForSubscriber(mock_pub[i]));

    const double scale_factor1 = 10;
    const double scale_factor2 = 2;

    mock_pub[i].publish(createCameraInfoMsg(scale_factor1, scale_factor2));
    ASSERT_TRUE(mock_sub.spinUntilMessages());

    sensor_msgs::CameraInfo correctResponse =
        cameraInfoTransformedMsg(output_width, output_height, scale_factor1, scale_factor2);
    sensor_msgs::CameraInfo response = mock_sub.front();  
    EXPECT_NEAR(correctResponse.width, response.width, 1E-100);
    EXPECT_NEAR(correctResponse.height, response.height, 1E-100);

    for (int i = 0; i < 9; i++)
    {
      EXPECT_NEAR(correctResponse.K[i], response.K[i], 1E-100);
    }
    for (int i = 0; i < 12; i++)
    {
      EXPECT_NEAR(correctResponse.P[i], response.P[i], 1E-100);
    }
  }
}

int main(int argc, char** argv)

{
  ros::init(argc, argv, "test_segmented_camera");
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}