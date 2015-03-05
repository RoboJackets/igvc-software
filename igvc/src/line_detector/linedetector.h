#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <flycapture/FlyCapture2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class LineDetector
{
public:
    LineDetector(ros::NodeHandle &handle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    bool isWorking();
    void onImageEvent();

private:
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    float getAvg(void);
    void blackAndWhite(float totalAvg);
    int display_dst(int delay);
    void detectObstacle(int i, int j);
    
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    void Erosion();
    void Dilation();

    void transformPoints(cv::Mat &src, cv::Mat &dst);
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);

    /** @brief the VideoCapture of the image/video */
    cv::VideoCapture cap;

    //For the erosion/dilation stuff
    /**
     * @brief contains the number corresponding to the element used for erosion
     * @note 2 is what we are currently using (an ellipse)
     */
    int erosion_elem;
    /**
     * @brief specifies the size of the area to be eroded.
     **/
    int erosion_size;
    int dilation_elem;
    int dilation_size;

    const int max_elem;
    const int max_kernel_size;
    cv::Mat p, pcam, transformMat;

    /**
     * @brief gaussian_size The size of the Gaussian blur. The bigger the greater the blur
     * @note Must be odd!
     */
    const int gaussian_size;

    /**
     * @brief src contains the original, unprocessed image
     */
    cv::Mat src;
    /**
     * @brief dst contains the new, processed image that isolates the lines
     */
    cv::Mat *dst;

    // ROS COMMUNICATION
	image_transport::ImageTransport _it;
	image_transport::Publisher _filt_img;
	image_transport::Subscriber _src_img;
};
#endif // LINEDETECTOR_H
