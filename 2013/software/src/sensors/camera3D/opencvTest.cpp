#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <flycapture/FlyCapture2.h>
#include "Bumblebee2.h"

using namespace cv;
using namespace FlyCapture2;

main()
{
   std::cout << "at least this will print";
    Bumblebee2 thisguy;
    thisguy.run();
    std::cout << "this shouldn't hit";


}
