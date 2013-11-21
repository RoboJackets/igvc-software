#ifndef CAMERAADAPTER_H
#define CAMERAADAPTER_H

#include <QWidget>
#include <QRect>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cameraadaptertester.h"
#include "common/events/Event.hpp"



namespace Ui {
class CameraAdapter;
}

class CameraAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit CameraAdapter(CameraAdapterTester *camera, QWidget *parent = 0);
    ~CameraAdapter();

protected:
    void paintEvent(QPaintEvent *e);
   // void resizeEvent(QResizeEvent *e);

private:
    Ui::CameraAdapter *ui;

    //Camera *camera;
    CameraAdapterTester *_camera;

    CameraData _data;
    bool gotData;

    boost::mutex _mutex;

    QImage CVMat2QImage(cv::Mat img);

    void OnCameraData(CameraData data);
    LISTENER(CameraAdapter, OnCameraData, CameraData)
};

#endif // CAMERAADAPTER_H
