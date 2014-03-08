#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <common/events/Event.hpp>
#include <common/datastructures/ImageData.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>


class LineDetector
{
public:
    LineDetector(Event<ImageData> &evtSrc);
    void onImageEvent(ImageData imgd);
    LISTENER(LineDetector, onImageEvent, ImageData)
    Event<ImageData> onNewLines;
    //Event<pcl::PointCloud> onNewCloud;
    pcl::PointCloud<pcl::PointXYZ> cloud;
private:
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    void transformPoints();
    void toPointCloud();
    float getAvg(void);
    void blackAndWhite(float totalAvg);
    int display_dst(int delay);
    void detectObstacle(int i, int j);

    void Erosion();
    void Dilation();

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
    cv::Mat p, pcam, transformDst, transformMat;

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
    cv::Mat dst;
};
#endif // LINEDETECTOR_H
