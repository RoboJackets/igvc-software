#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDateTime>

CameraAdapter::CameraAdapter(std::shared_ptr<StereoSource> source, QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    if(source.get() != nullptr)
    {
        _stereoSource = source;
        connect(_stereoSource.get(), SIGNAL(onNewData(StereoImageData)), this, SLOT(onCameraData(StereoImageData)));
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
    if(_stereoSource.get() != nullptr)
        disconnect(_stereoSource.get(), SIGNAL(onNewData(StereoImageData)), this, SLOT(onCameraData(StereoImageData)));
    delete ui;
}

void CameraAdapter::onCameraData(StereoImageData data)
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
    leftImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_left.jpg");
}

void CameraAdapter::on_saveRight_clicked()
{
    rightImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_right.jpg");
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    if(gotData){
        leftImage = CVMat2QImage(_data.left().mat());
        rightImage = CVMat2QImage(_data.right().mat());
        ui->leftFeedLabel->setPixmap(QPixmap::fromImage(leftImage));
        ui->rightFeedLabel->setPixmap(QPixmap::fromImage(rightImage));
    }

    _mutex.unlock();
    QWidget::paintEvent(e);
}
