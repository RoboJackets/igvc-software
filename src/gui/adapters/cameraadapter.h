#ifndef CAMERAADAPTER_H
#define CAMERAADAPTER_H

#include <QWidget>
#include <QRect>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hardware/sensors/camera/StereoSource.hpp>
#include <intelligence/linedetection/linedetector.h>
#include <intelligence/barrelfinder/barrelfinder.h>


namespace Ui {
class CameraAdapter;
}

/*!
 * \brief Widget for displaying stereo camera data.
 * \author Idan Mintz
 */
class CameraAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit CameraAdapter(QWidget *parent = 0);
    ~CameraAdapter();

protected:
    void paintEvent(QPaintEvent *e);

public slots:
    void newLeftCamImg(ImageData data);
    void newRightCamImg(ImageData data);

private slots:
    void on_saveLeft_clicked();

    void on_saveRight_clicked();

private:
    Ui::CameraAdapter *ui;

    QImage leftImage;
    QImage rightImage;

    ImageData leftData;
    ImageData rightData;

    boost::mutex _mutex;

    QImage CVMat2QImage(cv::Mat img);
};

#endif // CAMERAADAPTER_H
