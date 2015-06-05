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
#include <vector>

class LineDetector
{
public:
    LineDetector(ros::NodeHandle &handle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    bool isWorking();
    bool primaryMethod();
	bool otherMethod(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr cv_ptr);

private:
//    void findLines(cv_bridge::Cv_ImagePtr cv_ptr);
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    void blackoutSection(cv::Mat &imgmat, int rowl, int rowu, int coll, int colu);
    float getAvg(void);
    float getAvg(cv::Mat &imgmat);
    void blackAndWhite(float totalAvg);
    int display_dst(int delay);
    void detectObstacle(int i, int j, cv::Mat* dst);
    
    void img_callback(const sensor_msgs::ImageConstPtr& msg);

    void Erosion(cv::Mat* dst);
    void Dilation(cv::Mat* dst);

    void transformPoints(cv::Mat &src, cv::Mat &dst);
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);

    void backgroundSubtractor();
    void houghLines();
    void horzVertEdgeDet();
	void blobFinder();
    void helperSearch(void *totvis, void *input, int i, int j, cv::Mat *imgmat, int thresh);
    bool helperContains(std::tuple<int, int> pos, void *vis);
	bool adjWhite(int i, int j, cv::Mat *img, int thresh);
    bool InNature(cv::Mat& img, int r, int c, int size);
    void Dilation();

    void drawWhite(cv::Mat &image);
    cv::Mat applyFilter(cv::Mat &image, const cv::Mat &kernal);
    cv::Mat getGeometricMean(cv::Mat &img, cv::Mat &kernal);
    void subtractOrthog(std::vector<cv::Mat>& images);
    void RemoveNonMax(std::vector<cv::Mat>& images);
    void EnforceContinuity(std::vector<cv::Mat>& directions, cv::Mat& out);
    void binary(cv::Mat& src, cv::Mat& dst);

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
	ros::Publisher _line_cloud;
    image_transport::Publisher _filt_img1;
    image_transport::Publisher _filt_img2;
    image_transport::Publisher _filt_img3;
    image_transport::Publisher _filt_img4;
    image_transport::Publisher _filt_img5;
    image_transport::Publisher _filt_img6;
    image_transport::Publisher _filt_img7;
    image_transport::Publisher _filt_img8;
	image_transport::Subscriber _src_img;
};
#endif // LINEDETECTOR_H
