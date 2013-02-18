#include "lidardisplaywidget.h"
#include "qpainter.h"
#include <iostream>
#include <fstream>

LidarDisplayWidget::LidarDisplayWidget(QWidget *parent) :
    QWidget(parent),
    LonNewLidarData(this),
    _lidObstExtractor(),
    _obstacles()
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

    painter.setPen(Qt::black);
    boost::mutex::scoped_lock(_locker);
    for(int i = 0; i < 1024; i++)
    {
        LidarPoint p = _lidarData.points[i];
        if(p.valid)
        {
            QPointF end(cos(p.angle)*p.distance * _scale + origin.x(), -sin(p.angle)*p.distance*_scale + origin.y());

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

    painter.setPen(Qt::green);
//    std::cout << _obstacles.size() << std::endl;
    for(std::vector<Obstacle*>::iterator iter = _obstacles.begin(); iter < _obstacles.end(); iter++)
    {
        Obstacle *obstacle = *iter;
        Point *points = obstacle->getPoints();
        for(int i = 0; i < obstacle->getNumPoints()-1; i++)
        {
            Point p1 = points[i], p2 = points[i+1];
            QPointF start(p1.x, -p1.y);
            start *= _scale;
            start += origin;
            QPointF end(p2.x, -p2.y);
            end *= _scale;
            end += origin;
            painter.drawLine(start, end);
        }
    }

    painter.setPen(Qt::red);
    painter.drawLine(origin, origin + QPointF(_scale,0));
    painter.drawLine(origin, origin + QPointF(0,-_scale));

    painter.end();
    _drawing = false;
}

void LidarDisplayWidget::onNewLidarData(LidarState state)
{
    while(_drawing);
    boost::mutex::scoped_lock(_locker);
    _lidarData = state;
    for(int i = 0; i < 1024; i++)
    {
        if( abs( _lidarData.points[i].angle + M_PI / 2.0 ) < 0.75 )
            _lidarData.points[i].valid = false;
    }
    _obstacles = _lidObstExtractor.extractObstacles(_lidarData);
    update();
}

void LidarDisplayWidget::setScale(double scale)
{
    while(_drawing);
    _scale = scale;
    this->update();
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
