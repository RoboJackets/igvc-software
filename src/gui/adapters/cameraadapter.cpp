#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <opencv2/imgproc/imgproc.hpp>

CameraAdapter::CameraAdapter(StereoSource *source, QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter),
     LOnCameraData(this)
{
    ui->setupUi(this);

    if(source)
    {
        _stereoSource = source;
        _stereoSource->onNewData += &LOnCameraData;
    }

    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
    gotData = false;
    connect(ui->saveLeft,SIGNAL(released()),SLOT(on_saveLeft_clicked()));
}

CameraAdapter::~CameraAdapter()
{
    if(_stereoSource)
        _stereoSource->onNewData -= &LOnCameraData;
    delete ui;
}

void CameraAdapter::OnCameraData(StereoImageData data)
{

    if(isVisible())
    {
        _mutex.lock();
        _data = data;
        gotData = true;
        _mutex.unlock();
        update();
    }
}


QImage CameraAdapter::CVMat2QImage(cv::Mat img)
{
    cv::Mat temp(img.cols, img.rows, img.type());
    cv::cvtColor(img, temp, CV_BGR2RGB);

    QImage dest((uchar*)temp.data,temp.cols,temp.rows, temp.step, QImage::Format_RGB888);
    QImage dest2(dest);
    dest2.detach();
    return dest2;
}

void CameraAdapter::on_saveLeft_clicked()
{
    leftImage.save("../leftCapture.jpg");
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    if(gotData){
        leftImage = CVMat2QImage(_data.left().mat());
        ui->leftFeedLabel->setPixmap(QPixmap::fromImage(leftImage));
        ui->rightFeedLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.right().mat())));
//       ui->depthMapLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.depthMap)));
//       ui->pointCloudLabel->setPixmap(QPixmap::fromImage( CVMat2QImage(_data.pointCloud)));

    }


    _mutex.unlock();
    QWidget::paintEvent(e);
}
