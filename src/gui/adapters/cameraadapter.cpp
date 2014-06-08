#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDateTime>


CameraAdapter::CameraAdapter(std::shared_ptr<StereoSource> source, std::shared_ptr<LineDetector> source2, QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    connect(ui->saveLeft,SIGNAL(released()),SLOT(on_saveLeft_clicked()));
}

CameraAdapter::~CameraAdapter()
{
    if(_stereoSource.get() != nullptr)
        disconnect(_stereoSource.get(), SIGNAL(onNewData(StereoImageData)), this, SLOT(onCameraData(StereoImageData)));
    delete ui;
}

void CameraAdapter::newLeftCamImg(ImageData data)
{
    if(isVisible())
    {
        _mutex.lock();
        leftData = data;
        leftImage = CVMat2QImage(data.mat());
        gotData = true;
        _mutex.unlock();
        update();
    }
}

void CameraAdapter::newLineImage(cv::Mat data)
{
    if(isVisible())
    {
        _mutex.lock();
        lineData = data;
        lineImage = CVMat2QImage(data);
        _mutex.unlock();
    }
}

void CameraAdapter::newBarrelImage(cv::Mat data)
{
    if(isVisible())
    {
        _mutex.lock();
        barrelData = data;
        barrelImage = CVMat2QImage(data);
        _mutex.unlock();
    }
}

QImage CameraAdapter::CVMat2QImage(cv::Mat img)
{
    cv::Mat temp(img.cols, img.rows, img.type());
    if(img.channels() == 3 || img.channels() == 4)
        cv::cvtColor(img, temp, CV_BGR2RGB);
    else if(img.channels() == 1)
        cv::cvtColor(img, img, CV_GRAY2RGB);

    QImage dest((uchar*)temp.data,temp.cols,temp.rows, temp.step, QImage::Format_RGB888);
    QImage dest2(dest);
    dest2.detach();
    return dest2;
}

void CameraAdapter::on_saveLeft_clicked()
{
    //QDir().mkdir("../Images/");
    //leftImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_left.jpg");
    _mutex.lock();
    std::cout << cv::imwrite((QDateTime::currentDateTime().toString()+"img.jpg").toStdString(), _data.leftMat()) << std::endl;
    _mutex.unlock();
}

void CameraAdapter::on_saveRight_clicked()
{
    QDir().mkdir("../Images/");
    lineImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_right.jpg");
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    ui->leftFeedLabel->setPixmap(QPixmap::fromImage(leftImage));
    ui->rightFeedLabel->setPixmap(QPixmap::fromImage(rightImage));

    _mutex.unlock();
    QWidget::paintEvent(e);
}
