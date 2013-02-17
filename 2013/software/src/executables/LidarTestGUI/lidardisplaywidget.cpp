#include "lidardisplaywidget.h"
#include "qpainter.h"
#include <iostream>
#include <fstream>

LidarDisplayWidget::LidarDisplayWidget(QWidget *parent) :
    QWidget(parent),
    LonNewLidarData(this)
{
    _scale = 1.0;
    _drawing = false;
    _vMode = Lines;
}

void LidarDisplayWidget::paintEvent(QPaintEvent *)
{
    _drawing = true;
    QPainter painter;
    painter.begin(this);
    QPointF origin(this->width()/2.0, this->height()/2.0);
    boost::mutex::scoped_lock(_locker);
    for(int i = 0; i < 1024; i++)
    {
        LidarPoint p = _lidarData.points[i];
        if(p.valid)
        {
//            if( ( p.angle - M_PI ) < 0.1 )
//                std::cout <<  << std::endl;
            QPointF end(cos(p.angle)*p.distance*_scale + origin.x(), -sin(p.angle)*p.distance*_scale + origin.y());
//            std::cout << end.x() << ", " << end.y() << std::endl;
            switch(_vMode)
            {
            case Points:
                painter.drawPoint(end);
                break;
            case Lines:
                painter.drawLine(origin, end);
                break;
            }
        }
    }
    painter.end();
    _drawing = false;
}

void LidarDisplayWidget::onNewLidarData(LidarState state)
{
    while(_drawing);
    boost::mutex::scoped_lock(_locker);
    _lidarData = state;
    update();
}

void LidarDisplayWidget::setScale(double scale)
{
    while(_drawing);
    _scale = scale;
}

void LidarDisplayWidget::setLidar(Lidar *device)
{
    if(device)
    {
        _lidar = device;
        _lidar->onNewData += &LonNewLidarData;
    }
}

void LidarDisplayWidget::setViewMode(ViewMode mode)
{
    _vMode = mode;
    this->update();
}

double LidarDisplayWidget::scale()
{
    return _scale;
}

void LidarDisplayWidget::capture()
{
    boost::mutex::scoped_lock(_locker);
    using namespace std;
    ofstream file;
    file.open("lidar_frame.csv");
    file << "Index,Valid,Angle,Raw,Distance,Intensity\n";
    for(int i = 0; i < 1024; i++)
    {
        LidarPoint &pt = _lidarData.points[i];
        file << i << "," << pt.valid << "," << pt.angle << ","
             << pt.raw << "," << pt.distance << ","
             << (int)pt.intensity << "\n";
    }
    file.close();
}
