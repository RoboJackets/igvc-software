#include "segmented_camera_info_publisher.h"





//constructor
SegmentedCameraInfoPublisher::SegmentedCameraInfoPublisher() : pNh{"~"} 
{

    //TODO add .launch file and its params

    // cameras to obtain images from
    std::vector<std::string> camera_names;
    assertions::getParam(pNh, "camera_names", camera_names);


    std::vector<std::string> semantic_prefixes;
    assertions::getParam(pNh, "semantic_info_topic_prefix", semantic_prefixes);
    std::vector<std::string> semantic_suffixes;
    assertions::getParam(pNh, "semantic_info_topic_suffix", semantic_suffixes);

    //assertions::getParam(pNh, "image_info_base_topic", image_info_base_topic);

    assertions::getParam(pNh, "output_width", output_width);
    assertions::getParam(pNh, "output_height", output_height);

    assertions::getParam(pNh, "segmented_publisher_path", seg_cam_path);


    //publish for each cam
    for (size_t i = 0; i < camera_names.size(); i++)
    {
        auto camera_name = camera_names[i];
        auto prefix = semantic_prefixes[i]; //prefix is topic
        auto suffix = semantic_suffixes[i];

        std::string semantic_info_topic = prefix + camera_name;
        semantic_info_topic.append(suffix);

        // subscriber, also calls scalecamerainfo
        subs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(semantic_info_topic, 10, boost::bind(&SegmentedCameraInfoPublisher::ScaleCameraInfo, this, _1, output_width, output_height, camera_name) ) );

        //ie subscribe name/raw/info -> name/segmented/info
        ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>(camera_name + seg_cam_path, 1);
        //g_pubs[camera_name] = info_pub;
        g_pubs.insert ( std::pair<std::string, ros::Publisher>(camera_name,info_pub) );
        
    }
}

void SegmentedCameraInfoPublisher::ScaleCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info, double width, double height, std::string camera_name) 
{
    sensor_msgs::CameraInfo changed_camera_info = *camera_info;

    double w_ratio = static_cast<double>(width) / static_cast<double>(camera_info->width);
    double h_ratio = static_cast<double>(height) / static_cast<double>(camera_info->height);

    changed_camera_info.width = static_cast<unsigned int>(width);
    changed_camera_info.height = static_cast<unsigned int>(height);

    changed_camera_info.K = { { camera_info->K[0] * w_ratio, 0, camera_info->K[2] * w_ratio, 0, camera_info->K[4] * h_ratio,
                                camera_info->K[5] * h_ratio, 0, 0, 1 } };
    changed_camera_info.P = { { camera_info->P[0] * w_ratio, 0, camera_info->P[2] * w_ratio, 0, 0,
                                camera_info->P[5] * h_ratio, camera_info->P[6] * h_ratio, 0, 0, 0, 1, 0 } };

    //g_pubs[camera_name] = changed_camera_info;
    g_pubs.at(camera_name).publish(changed_camera_info);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "segmented camera info publisher");
    SegmentedCameraInfoPublisher segmented_camera_info_publisher;
    //spin
    ros::spin();
    return 0;
}
