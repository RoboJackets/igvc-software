#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <flycapture/FlyCapture2.h>

using namespace cv;
using namespace FlyCapture2;

int main(int argc, char** argv)
{
    Mat m = imread("/home/alex/Desktop/L5bOZ.jpg",1);
    Mat gray_image;
    cvtColor( m, gray_image, CV_BGR2GRAY);
    int nRows, nCols;
    nCols = m.cols*m.channels();
    std::cout << m.channels();
    nRows = m.rows;
    unsigned char* wholeData = new unsigned char[nCols*nRows];
    if (m.isContinuous())
    {
        nCols *= nRows;
        nRows=1;
    }
    int i,j;
    unsigned char *ptr;
    for (i=0;i<nRows;i++)
    {
        ptr = m.ptr<unsigned char>(i);
        for(j=0;j<nCols;j++)
        {
            wholeData[i*nCols + j] = ptr[j];
            //ptr[j]=255;
        }
    }
    //This correctly recreates the matrix using the data. Now do the same for point grey image and use this to change it back
    //
    Mat mn(m.rows, m.cols,CV_8UC3, wholeData,Mat::AUTO_STEP);

    FlyCapture2::Image thisguy(wholeData, 3);
    FlyCapture2::Image newGuy(m.rows, m.cols, m.cols, wholeData,3,PIXEL_FORMAT_BGR, NONE);
    newGuy.Save("/home/alex/Desktop/thisImage.bmp");


    //imwrite("/home/alex/Desktop/thisImage.png",mn);

    delete[] wholeData;
}





/*
int main()
{
    Mat m = imread("/home/alex/Desktop/L5bOZ.jpg",1);
    int size = m.rows*m.cols*m.channels();
    std::cout << m.channels() << std::endl;
    unsigned char* ptr = m.ptr<unsigned char>(0);
    int i, ncols;
    for
    for i=0;i<ncols;i++)
    std::cout << (int)ptr[1] << std::endl;

    //FlyCapture2::Image img(m.);

}

*/

FlyCapture2::Image * ImageFromFile(char* fileName)
{


}
