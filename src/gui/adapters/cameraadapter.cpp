#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDateTime>


CameraAdapter::CameraAdapter(QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    connect(ui->saveLeft,SIGNAL(released()),SLOT(on_saveLeft_clicked()));
}

CameraAdapter::~CameraAdapter()
{
    delete ui;
}

void CameraAdapter::newLeftCamImg(ImageData data)
{
    if(isVisible())
    {
        _mutex.lock();
        leftData = data;
        leftImage = CVMat2QImage(data.mat());
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
        update();
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
    _mutex.lock();
    cv::imwrite((QDateTime::currentDateTime().toString()+"img.jpg").toStdString(),  leftData.mat());
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
    ui->rightFeedLabel->setPixmap(QPixmap::fromImage(lineImage));
    ui->barrelImg_label->setPixmap(QPixmap::fromImage(barrelImage));

    _mutex.unlock();
    QWidget::paintEvent(e);
}
