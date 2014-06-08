#include "barrelfinder.h"
#include <common/logger/logger.h>
#include <common/config/configmanager.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <common/utils/ImageUtils.h>

BarrelFinder::BarrelFinder(QObject *parent) :
    QObject(parent)
{
}

void BarrelFinder::onNewImage(ImageData data)
{
    cloud.clear();

    if(data.mat().channels() != 3)
        Logger::Log(LogLevel::Error, "Barrel finder can only work on BGR images!");

    cv::Mat img = data.mat();
    cv::Mat binary = cv::Mat(img.rows, img.cols, CV_8UC1);

    int bT = ConfigManager::Instance().getValue("BarrelFinder", "blueThresh", 100);
    int gT = ConfigManager::Instance().getValue("BarrelFinder", "greenThresh", 100);
    int rT = ConfigManager::Instance().getValue("BarrelFinder", "redThresh", 150);

    for(int r = 0; r < img.rows; r++)
    {
        const uchar* row = img.ptr<uchar>(r);
        uchar* rowBin = binary.ptr<uchar>(r);
        for(int c = 0; c < img.cols*3; c+=3)
        {
            uchar b = row[c];
            uchar g = row[c+1];
            uchar r = row[c+2];

            if(r > rT && b < bT && g < gT)
                rowBin[c/3] = 255;
            else
                rowBin[c/3] = 0;
        }
    }

    { // Erosion
        int erosion_type, erosion_elem = 2, erosion_size = 2;
        if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
        else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
        else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

        cv::Mat element = cv::getStructuringElement( erosion_type,
                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             cv::Point( erosion_size, erosion_size ) );
        // Apply the erosion operation
        cv::erode( binary, binary, element);
    }

    { // Dilation
        int dilation_type, dilation_elem = 2, dilation_size = 1;
        if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
        else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
        else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }

        cv::Mat element = cv::getStructuringElement( dilation_type,
                                             cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                             cv::Point( dilation_size, dilation_size ) );
        // Apply the dilation operation
        cv::dilate( binary, binary, element );
    }


    int minWidth = ConfigManager::Instance().getValue("BarrelFinder", "minWidth", 50);
    for(int r = 0; r < binary.rows; r++)
    {
        uchar* row = binary.ptr<uchar>(r);
        int lastRising = 0;
        for(int c = 1; c < binary.cols; c++)
        {
            if(row[c] && !row[c-1]) // Rising
            {
                lastRising = c;
            }
            else if(!row[c] && row[c-1]) // Falling
            {
                if(c - lastRising < minWidth)
                    for(int i = lastRising; i <= c; i++)
                        row[i] = 0;
            }
        }
    }

    // Transpose to optimize vertical processing
    cv::transpose(binary, binary);

    for(int r = 0; r < binary.rows; r++)
    {
        uchar* row = binary.ptr<uchar>(r);
        int lastFalling = -1;
        for(int c = 1; c < binary.cols; c++)
        {
            if(row[c] && !row[c-1]) // Rising
            {
                if(lastFalling > -1)
                    for(int i = lastFalling; i <= c; i++)
                        row[i] = 255;
                lastFalling = -1;
            }
            else if(!row[c] && row[c-1]) // Falling
            {
                lastFalling = c;
            }
        }
    }

    int minHeight = ConfigManager::Instance().getValue("BarrelFinder", "minHeight", 75);

    for(int r = 0; r < binary.rows; r++)
    {
        uchar* row = binary.ptr<uchar>(r);
        int lastRising = -1;
        for(int c = 1; c < binary.cols; c++)
        {
            if(row[c] && !row[c-1]) // Rising
            {
                lastRising = c;
            }
            else if(!row[c] && row[c-1]) // Falling
            {
                if(c - lastRising < minHeight)
                    for(int i = lastRising; i <= c; i++)
                        row[i] = 0;
            }
        }
    }


    for (int r =0; r<binary.rows; r++){
        uchar* row = binary.ptr<uchar>(r);
        for (int c=0; c<binary.cols; c++){
            if (row[c] && !row[c+1]){
                row[c]=255;
            }
            else
            {
                row[c]=0;
            }
        }
    }

    // Transpose back to correct orientation
    cv::transpose(binary, binary);
    cv::rectangle(binary, cv::Point(binary.cols/2, binary.rows), cv::Point(binary.cols/2 + 30, binary.rows - 30), cv::Scalar(255));
    cv::Mat dst;
    cv::cvtColor(binary, dst, CV_GRAY2BGR);
    transformPoints(binary, dst);
    cloud = toPointCloud(binary);
    onNewLinesMat(binary);


    cv::imshow("Barrel Finder", dst);

    newCloudFrame(cloud.makeShared());
}




