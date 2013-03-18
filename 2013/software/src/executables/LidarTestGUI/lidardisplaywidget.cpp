#include "lidardisplaywidget.h"
#include <QPainter>
#include <QEvent>
#include <QMouseEvent>
#include <iostream>
#include <fstream>

LidarDisplayWidget::LidarDisplayWidget(QWidget *parent) :
    QWidget(parent),
    LonNewLidarData(this),
    LonNewObstacleData(this)
{
    _scale = 1.0;
    _vMode = Lines;
    _origin = QPointF(164, 104.5);
    shouldUpdateOnScaling = true;
    showInvalid = false;
}

void LidarDisplayWidget::paintEvent(QPaintEvent *)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    QPainter painter;
    painter.begin(this);

    painter.setPen(Qt::black);
    for(int i = 0; i < 1024; i++)
    {
        LidarPoint p = _lidarData.points[i];
        if(p.valid)
        {
            QPointF end(cos(p.angle)*p.distance * _scale + _origin.x(), -sin(p.angle)*p.distance*_scale + _origin.y());

            switch(_vMode)
            {
            case Points:
                painter.drawPoint(end);
                break;
            case Lines:
                painter.drawLine(_origin, end);
                break;
            }
        } else {
            if(showInvalid)
            {
                painter.setPen(Qt::gray);
                QPointF end(cos(p.angle)*p.distance * _scale + _origin.x(), -sin(p.angle)*p.distance*_scale + _origin.y());

                switch(_vMode)
                {
                case Points:
                    painter.drawPoint(end);
                    break;
                case Lines:
                    painter.drawLine(_origin, end);
                    break;
                }
                painter.setPen(Qt::black);
            }
        }
    }

    if(!_obstacles.empty())
    {
        painter.setPen(QPen(Qt::blue, 2));
        for(std::vector<Obstacle*>::iterator iter = _obstacles.begin(); iter < _obstacles.end(); iter++)
        {
            Obstacle *obstacle = *iter;
            Point *points = obstacle->getPoints();
            for(int i = 0; i < obstacle->getNumPoints()-1; i++)
            {
                Point &p1 = points[i], &p2 = points[i+1];
                QPointF start(p1.x, -p1.y);
                start *= _scale;
                start += _origin;
                QPointF end(p2.x, -p2.y);
                end *= _scale;
                end += _origin;
                painter.drawLine(start, end);
            }
        }
    }

    painter.setPen(Qt::red);
    painter.drawLine(_origin, _origin + QPointF(_scale,0));
    painter.drawLine(_origin, _origin + QPointF(0,-_scale));

    painter.end();
}

void LidarDisplayWidget::onNewLidarData(LidarState state)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    _lidarData = state;
    update();
}

void LidarDisplayWidget::onNewObstacleData(std::vector<Obstacle*> obstacles)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    _obstacles = obstacles;
    update();
}

void LidarDisplayWidget::setScale(double scale)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    _scale = scale;
    // Normally, I wouldn't recommend a hack like this,
    // but for some reason this causes things to crash
    // when running on the NAV200, so I just have the
    // main window tell this widget not to update on
    // scaling when running on the NAV200. You shouldn't
    // notice much of a difference when running the app
    // because the NAV200 refreshes the data often enough
    // to more or less keep up.
    if(shouldUpdateOnScaling)
        this->update();
}

void LidarDisplayWidget::setLidar(Lidar *device)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    if(_lidObstExractor)
    {
        _lidObstExractor->disconnect();
        delete _lidObstExractor;
    }
    if(device)
    {
        _lidar = device;
        _lidar->onNewData += &LonNewLidarData;
        _lidObstExractor = new LidarObstacleExtractor(_lidar);
        _lidObstExractor->onNewData += &LonNewObstacleData;
    }
}

bool LidarDisplayWidget::event(QEvent *event)
{
    if(event->type() == QEvent::MouseButtonPress)
    {
        _isDragging = true;
        return true;
    } else if(event->type() == QEvent::MouseButtonRelease)
    {
        _isDragging = false;
        return true;
    } else if(event->type() == QEvent::MouseMove)
    {
        if(_isDragging)
        {
            QMouseEvent *me = static_cast<QMouseEvent *>(event);
            _origin.setX(me->x());
            _origin.setY(me->y());
            // See setScale() to find out why this is here.
            if(shouldUpdateOnScaling)
                this->update();
        }
        return true;
    }
    return QWidget::event(event);
}

void LidarDisplayWidget::clearDeviceListeners()
{
    if(_lidar)
    {
        _lidar->onNewData -= &LonNewLidarData;
        _lidar = 0;
    }
}

void LidarDisplayWidget::setViewMode(ViewMode mode)
{
    boost::unique_lock<boost::mutex> lock(_locker);
    _vMode = mode;
    this->update();
}

double LidarDisplayWidget::scale()
{
    return _scale;
}

void LidarDisplayWidget::capture()
{
    boost::unique_lock<boost::mutex> lock(_locker);
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
