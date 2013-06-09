#ifndef PERSPECTIVETRANSFORMER_HPP_INCLUDED
#define PERSPECTIVETRANSFORMER_HPP_INCLUDED

#include <sensors/camera3D/StereoSource.hpp>
#include <events/Event.hpp>

class PerspectiveTransformer
{
public:
    PerspectiveTransformer(StereoSource *source)
        : LProcessFrame(this)
    {
        _source = source;
        _source->onNewData += &LProcessFrame;

        FileStorage file("/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml", FileStorage::READ);
        file["mapMatrix"] >> _transform;
        file.release();
    }

    Event<Mat> OnNewTransformedImage;

private:
    StereoSource* _source;
    Mat _transform;

    void ProcessFrame(StereoImageData data)
    {
        std::cout << "PerspTrans procFrame" << std::endl;
        using namespace cv;
        _source->LockImages();
        try
        {

            Mat frame = data.leftMat();

            resize(frame, frame, Size(640,480));

            Mat warped;

            warpPerspective(frame, warped, _transform, Size(1280,960));

            resize(warped, warped, Size(640, 480));

            OnNewTransformedImage(warped);
        }
        catch(...)
        {
        }
        _source->UnlockImages();
    }
    LISTENER(PerspectiveTransformer, ProcessFrame, StereoImageData);

};

#endif // PERSPECTIVETRANSFORMER_HPP_INCLUDED
