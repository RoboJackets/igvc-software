#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <QGridLayout>

CameraAdapter::CameraAdapter(QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    currentLeftFrame.load("/home/idan/Downloads/testImage.png");
    currentRightFrame.load("/home/idan/Downloads/testImage.png");
    currentDepthFrame.load("/home/idan/Downloads/testImage.png");
    currentPointCloudFrame.load("/home/idan/Downloads/testImage.png");

    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
}

CameraAdapter::~CameraAdapter()
{
    /*if(_joystick)
        _joystick->onNewData -= &LOnJoystickData;
    delete ui;*/
}

void CameraAdapter::OnCameraData(/*CameraState state*/)
{
    if(isVisible())
    {
        _mutex.lock();
        //_state = state;
        _mutex.unlock();
        update();
    }
}

void CameraAdapter::paint(QPainter *painter)
{
    //painter->drawImage(ui->leftFeedTab, currentLeftFrame);
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{


    _mutex.lock();
    ui->leftFeedLabel->setPixmap(QPixmap::fromImage(currentLeftFrame));
    ui->rightFeedLabel->setPixmap(QPixmap::fromImage(currentRightFrame));
    ui->depthMapLabel->setPixmap(QPixmap::fromImage(currentDepthFrame));
    ui->pointCloudLabel->setPixmap(QPixmap::fromImage(currentPointCloudFrame));


    _mutex.unlock();
    QWidget::paintEvent(e);
}

/*void CameraAdapter::resizeEvent(QResizeEvent *e)
{
    QSize size = currentLeftFrame.size();
    cout << size.width() << ", " << size.height() << "\n";
    size.scale(this->size().boundedTo(size), Qt::KeepAspectRatio);
    currentLeftFrame = currentLeftFrame.scaled(size, Qt::KeepAspectRatio);

    this->resize(size);


    ui->tabWidget->resize(this->sizeHint());


    QWidget::resizeEvent(e);

}*/
