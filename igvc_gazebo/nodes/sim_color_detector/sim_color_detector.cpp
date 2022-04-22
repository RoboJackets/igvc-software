#include "sim_color_detector.hpp"

SimColorDetector::SimColorDetector() : pNh("~")
{
  assertions::getParam(pNh, "camera_names", camera_names);

  assertions::getParam(pNh, "semantic_topic_prefix", semantic_prefixes);
  assertions::getParam(pNh, "semantic_topic_suffix", semantic_suffixes);

  assertions::getParam(pNh, "image_base_topic", image_base_topic);

  assertions::getParam(pNh, "output_image/width", output_size.width);
  assertions::getParam(pNh, "output_image/height", output_size.height);

  assertions::getParam(pNh, "lines/lower/h", lower_lines[0]);
  assertions::getParam(pNh, "lines/lower/s", lower_lines[1]);
  assertions::getParam(pNh, "lines/lower/v", lower_lines[2]);
  assertions::getParam(pNh, "lines/upper/h", upper_lines[0]);
  assertions::getParam(pNh, "lines/upper/s", upper_lines[1]);
  assertions::getParam(pNh, "lines/upper/v", upper_lines[2]);

  assertions::getParam(pNh, "line_topic", line_topic);
  assertions::getParam(pNh, "barrel_topic", barrel_topic);

  // Insert subscribers and publishers
  for (size_t i = 0; i < camera_names.size(); ++i)
  {
    auto camera_name = camera_names[i];
    auto prefix = semantic_prefixes[i];
    auto suffix = semantic_suffixes[i];

    std::string semantic_topic = prefix + line_topic + suffix;

    // Subscribe to raw camera image
    image_transport::ImageTransport image_transport(nh);
    image_transport::CameraSubscriber cam_sub = image_transport.subscribeCamera(
        camera_name + image_base_topic, 1, boost::bind(&SimColorDetector::handleImage, this, _1, _2, camera_name));
    subs.push_back(cam_sub);

    // Publish line and barrel segmentation
    image_transport::CameraPublisher cam_pub = image_transport.advertiseCamera(semantic_topic, 1);
    ros::Publisher barrel_pub = nh.advertise<sensor_msgs::Image>(camera_name + barrel_topic, 1);
    LineBarrelPair publishers = { cam_pub, barrel_pub };

    pubs.insert(std::make_pair(camera_name, publishers));
  }
}

// Scales a CameraInfo message and modifies the message's K and P (matrix) values
sensor_msgs::CameraInfo SimColorDetector::scaleCameraInfo(const sensor_msgs::CameraInfo& camera_info, int width,
                                                          int height)
{
  sensor_msgs::CameraInfo changed_camera_info = camera_info;
  changed_camera_info.D = camera_info.D;
  changed_camera_info.distortion_model = camera_info.distortion_model;
  changed_camera_info.R = camera_info.R;
  changed_camera_info.roi = camera_info.roi;
  changed_camera_info.binning_x = camera_info.binning_x;
  changed_camera_info.binning_y = camera_info.binning_y;

  double w_ratio = static_cast<double>(width) / static_cast<double>(camera_info.width);
  double h_ratio = static_cast<double>(height) / static_cast<double>(camera_info.height);

  changed_camera_info.width = static_cast<unsigned int>(width);
  changed_camera_info.height = static_cast<unsigned int>(height);

  changed_camera_info.K = { { camera_info.K[0] * w_ratio, 0, camera_info.K[2] * w_ratio, 0, camera_info.K[4] * h_ratio,
                              camera_info.K[5] * h_ratio, 0, 0, 1 } };
  changed_camera_info.P = { { camera_info.P[0] * w_ratio, 0, camera_info.P[2] * w_ratio, 0, 0,
                              camera_info.P[5] * h_ratio, camera_info.P[6] * h_ratio, 0, 0, 0, 1, 0 } };

  return changed_camera_info;
}

void SimColorDetector::handleImage(const sensor_msgs::ImageConstPtr& msg,
                                   const sensor_msgs::CameraInfoConstPtr& camera_info, std::string camera_name)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat frame;  // Input image

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  // Creates copy of image data from ROS message
    frame = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV-Bridge error: %s", e.what());
    return;
  }

  cv::resize(frame, frame, output_size, 0, 0, cv::INTER_LINEAR);
  cv::Mat frame_hsv;
  cv::cvtColor(frame, frame_hsv, CV_BGR2HSV);

  cv::Mat output_lines(frame_hsv.rows, frame_hsv.cols, CV_8UC1, cv::Scalar::all(0));    // Output image lines (B&W)
  cv::Mat output_barrels(frame_hsv.rows, frame_hsv.cols, CV_8UC1, cv::Scalar::all(0));  // Output image barrels (B&W)

  // Segment lines by checking bounds
  cv::inRange(frame_hsv, lower_lines, upper_lines, output_lines);

  sensor_msgs::Image outmsg;
  outmsg.header = msg->header;
  cv_ptr->encoding = "mono8";

  // Modify camera info to fit scaled image
  sensor_msgs::CameraInfo modified_camera_info =
      scaleCameraInfo(*camera_info, output_size.width, output_size.height);

  // Publish line segmentation
  cv_ptr->image = output_lines;
  cv_ptr->toImageMsg(outmsg);
  pubs.at(camera_name).camera.publish(outmsg, modified_camera_info);

  // Publish barrel segmentation (blank for now until multiclass segmentation integrated)
  cv_ptr->image = output_barrels;
  cv_ptr->toImageMsg(outmsg);
  pubs.at(camera_name).barrel.publish(outmsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_color_detector");
  SimColorDetector sim_color_detector;
  ros::spin();
}