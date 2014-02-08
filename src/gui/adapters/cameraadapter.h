#ifndef CAMERAADAPTER_H
#define CAMERAADAPTER_H

#include <QWidget>
#include <QRect>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <hardware/sensors/camera/StereoSource.hpp>


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
    explicit CameraAdapter(StereoSource *source, QWidget *parent = 0);
    ~CameraAdapter();

protected:
    void paintEvent(QPaintEvent *e);
   // void resizeEvent(QResizeEvent *e);
private slots:
    void on_saveLeft_clicked();

private:
    Ui::CameraAdapter *ui;

    QImage leftImage;

    StereoSource *_stereoSource;

    StereoImageData _data;
    bool gotData;

    boost::mutex _mutex;

    QImage CVMat2QImage(cv::Mat img);

    void OnCameraData(StereoImageData data);
    LISTENER(CameraAdapter, OnCameraData, StereoImageData)
};

#endif // CAMERAADAPTER_H
