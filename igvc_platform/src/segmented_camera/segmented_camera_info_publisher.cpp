
#include "differential_drive.h"






SegmentedCameraInfoPublisher::SegmentedCameraInfoPublisher() : pNh{"~"} 
{

    // cameras to obtain images from
    std::vector<std::string> camera_names;
    assertions::getParam(pNh, "camera_names", camera_names);


    std::vector<std::string> semantic_prefixes;
    assertions::getParam(pNh, "semantic_info_topic_prefix", semantic_prefixes);
    std::vector<std::string> semantic_suffixes;
    assertions::getParam(pNh, "semantic_info_topic_suffix", semantic_suffixes);


    assertions::getParam(pNh, "image_info_base_topic", image_info_base_topic);

    assertions::getParam(pNh, "output_width", output_width);
    assertions::getParam(pNh, "output_height", output_height);

    for (size_t i = 0; i < camera_names.size(); i++)
    {
        auto camera_name = camera_names[i];
        auto prefix = semantic_prefixes[i];
        auto suffix = semantic_suffixes[i];

        std::string semantic_info_topic = prefix + line_topic;
        semantic_info_topic.append(suffix);

        // subscriber
        image_transport::CameraSubscriber cam_sub = image_transport.subscribeCamera(
            camera_name + image_info_base_topic, 1, boost::bind(ScaleCameraInfo, _1, _2, _3, camera_name));
        subs.push_back(cam_sub);

        // publish segented camera info
        image_transport::CameraPublisher cam_pub = image_transport.advertiseCamera(semantic_info_topic, 1);

        //TODO: figure out where we want to publish the info
        ros::Publisher info_pub = nh.advertise<sensor_msgs::Image>(camera_name + camera_info, 1);
    }
}

void SegmentedcameraInfoPublisher::ScaleCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, double width, double height, std::string camera_name) 
{
    //direct copypasta from nodes/sim_color_detector/main
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

    // publish segmented camera info
    g_pubs.at(camera_name).publish(changed_camera_info);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "segmented camera info publisher");
    SegmentedCameraInfoPublisher segmented_camera_info_publisher;
    //spin
    ros::spin();
}
