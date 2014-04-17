#include "pathadapter.h"
#include "ui_pathadapter.h"
#include <QPainter>
#include "intelligence/pathplanning/pathplanner.hpp"
#include <math.h>
#include "intelligence/pathplanning/astarplanner.h"
#include <common/utils/AngleUtils.h>
#include <QWheelEvent>

PathAdapter::PathAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PathAdapter),
    center(this->width()/2, this->height()/2)
{
    ui->setupUi(this);

    scale = 10;

    AStarPlanner planner;
    connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
    connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
    connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
    connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

    setStart(RobotPosition(0,0,0));
    setEnd(RobotPosition(10,10,0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
    for(int x = -10; x < 10; x++)
        map->push_back(pcl::PointXYZ(x,5,0));
//    map->push_back(pcl::PointXYZ(100,100,0));
    setMap(map);
    path = planner.GetPath();
}

PathAdapter::~PathAdapter()
{
    delete ui;
}

void PathAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    int centerX = center.x();
    int centerY = center.y();
    int height = painter.window().height();

    painter.setPen(QColor(127,127,127));
    painter.drawLine(0, height - centerY, painter.window().width(), height - centerY);
    painter.drawLine(centerX, 0, centerX, painter.window().height());

    for(auto point : path)
    {
        auto move = point.first;
        auto location = point.second;

        painter.setPen(Qt::black);
        if(fabs(move.W) < 1e-10)
        {
            // Draw straight line
            double x2 = location.x - (move.V * move.DeltaT * cos(M_PI_2 - location.theta));
            double y2 = location.y - (move.V * move.DeltaT * sin(M_PI_2 - location.theta));
            painter.drawLine(location.x * scale + centerX, height - (location.y * scale + centerY), x2 * scale + centerX, height - (y2 * scale + centerY));
        }
        else
        {
            // Draw arc
            drawArc(&painter, location, move, scale, center);
        }
    }

    painter.end();
}

void PathAdapter::drawArc(QPainter *painter, SearchLocation dest, SearchMove moveTaken, double scale, QPoint origin)
{
    double R = (moveTaken.V / moveTaken.W ) * scale;
    double cx = ( dest.x * scale ) - R * cos(M_PI - dest.theta);
    double cy = ( dest.y * scale ) - R * sin(M_PI - dest.theta);
    int scx = origin.x();
    int scy = origin.y();
    int height = painter->window().height();
    if(moveTaken.W > 0)
        for(double t = M_PI - dest.theta; t <= M_PI - dest.theta + ( moveTaken.W * moveTaken.DeltaT ); t += 0.001)
            painter->drawPoint(cx + R  * cos(t) + scx, height - (cy + R * sin(t) + scy));
    else
        for(double t = M_PI - dest.theta; t >= M_PI - dest.theta + ( moveTaken.W * moveTaken.DeltaT ); t -= 0.001)
            painter->drawPoint(cx + R  * cos(t) + scx, height - (cy + R * sin(t) + scy));
}

void PathAdapter::wheelEvent(QWheelEvent *event)
{
    scale += 0.01 * event->delta();
    scale = max(scale, 0.);
    scale = min(scale, 100.);
    this->update();
}

void PathAdapter::mouseMoveEvent(QMouseEvent *e)
{
    center.setX(e->x());
    center.setY(this->height() - e->y());
    this->update();
}
