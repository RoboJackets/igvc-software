#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


image_transport::Publisher _depth_pub;
ros::Publisher _pointcloud_pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const double fx = 855.667983;
const double fy = 857.549007;

void dispCallback(const stereo_msgs::DisparityImage& msg)
{
    auto focalLength = msg.f;
    auto basline = 0.102;
    std::vector<unsigned char> data = msg.image.data;
    sensor_msgs::Image depthImage = msg.image;
    depthImage.header.frame_id = "/left";

    PointCloud::Ptr cloud (new PointCloud);
    cloud->header.frame_id = "/left";
    cloud->height = msg.image.height;
    cloud->width = msg.image.width;

    int counter = 0;
    for(int i=0;i<msg.image.step*msg.image.height;i+=4)
    {
        float* t = (float*)&data[i];
        float d = *t;
        float Z = focalLength*basline/abs(d);
        unsigned char c[sizeof Z];
        memcpy(c, &Z, sizeof Z);
        depthImage.data[i] = c[0];
        depthImage.data[i+1] = c[1];
        depthImage.data[i+2] = c[2];
        depthImage.data[i+3] = c[3];

        if(d<msg.min_disparity || d>msg.max_disparity)
            Z=0;

        int x = counter%depthImage.width;
        int y = counter/depthImage.width;
        pcl::PointXYZ p;
        p.x = (x-depthImage.width/2)*(Z/fx);
        p.y = (y-depthImage.height/2)*(Z/fy);
        p.z = Z;
        cloud->points.push_back(p);
        counter++;
    }

    _depth_pub.publish(depthImage);
    _pointcloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disp");
    
    ros::NodeHandle nh;
    
	image_transport::ImageTransport _it(nh);

    _depth_pub = _it.advertise("/depthImage", 1);
    _pointcloud_pub = nh.advertise<PointCloud>("/pointcloud", 1);

    ros::Subscriber disp_sub = nh.subscribe("/disparity", 1, dispCallback);

	ros::spin();
}


