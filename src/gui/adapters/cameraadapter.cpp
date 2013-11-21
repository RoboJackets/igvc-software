#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <opencv2/imgproc/imgproc.hpp>

CameraAdapter::CameraAdapter(QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)/*,
     LOnCameraData(this)*/
{
    ui->setupUi(this);

//    if(camera)
//    {
//        _camera = camera;
//        _camera->onNewData += &LOnCameraData;
//    }
    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
    gotData = false;
}

CameraAdapter::~CameraAdapter()
{
//    if(_camera)
//        _camera->onNewData -= &LOnCameraData;
    delete ui;
}

//void CameraAdapter::OnCameraData(CameraData data)
//{

//    if(isVisible())
//    {
//        _mutex.lock();
//        _data = data;
//        gotData = true;
//        _mutex.unlock();
//        update();
//    }
//}


QImage CameraAdapter::CVMat2QImage(cv::Mat img)
{
    cv::Mat temp(img.cols, img.rows, img.type());
    cv::cvtColor(img, temp, CV_BGR2RGB);

    QImage dest((uchar*)temp.data,temp.cols,temp.rows, temp.step, QImage::Format_RGB888);
    QImage dest2(dest);
    dest2.detach();
    return dest2;
}


void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    if(gotData){

//       ui->leftFeedLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.leftFeed)));
//       ui->rightFeedLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.rightFeed)));
//       ui->depthMapLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.depthMap)));
//       ui->pointCloudLabel->setPixmap(QPixmap::fromImage( CVMat2QImage(_data.pointCloud)));

    }


    _mutex.unlock();
    QWidget::paintEvent(e);
}
