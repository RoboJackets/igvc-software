#ifndef TRANSFORMER_H
#define TRANSFORMER_H
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <common/events/Event.hpp>
#include <hardware/sensors/DataStructures/ImageData.hpp>

#include <common/logger/logger.h>
#include <sstream>
#include <iostream>
#include <cstdlib>


class transformer
{
public:
    transformer(Event<ImageData> &evtSrc);
    void onImageEvent(ImageData imgd);
    LISTENER(transformer, onImageEvent, ImageData);
    Event<ImageData> onNewLines;

private:
    cv::Mat src, dst;
    cv::Mat p, pcam, transformMat, addition;
};

#endif // TRANSFORMER_H
