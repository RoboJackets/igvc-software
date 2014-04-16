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
    ui(new Ui::PathAdapter)
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
//    for(int x = -10; x < 10; x++)
//        map->push_back(pcl::PointXYZ(x,5,0));
    map->push_back(pcl::PointXYZ(100,100,0));
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

    int centerX = painter.window().width()/2;
    int centerY = painter.window().height()/2;
    int height = painter.window().height();

    for(auto point : path)
    {
        auto move = point.first;
        auto location = point.second;
        std::cout << location.x << ", " << location.y << std::endl;

        //painter.drawEllipse(location.x * scale + centerX, height - (location.y * scale + centerY), 3, 3);

        painter.setPen(QColor(20,20,20));
        painter.drawLine(0, painter.window().height()/2, painter.window().width(), painter.window().height()/2);
        painter.drawLine(painter.window().width()/2, 0, painter.window().width()/2, painter.window().height());

        painter.setPen(Qt::blue);
        for(int i = -1; i <= 1; i++)
            for(int j = -1; j <= 1; j++)
                painter.drawPoint(location.x * scale + centerX + i, height - (location.y * scale + centerY) + j);
        painter.setPen(Qt::black);
        if(fabs(move.W) < 1e-10)
        {
            // Draw straight line
            double x2 = location.x + (move.V * move.DeltaT * cos(M_PI_2 - location.theta));
            double y2 = location.y + (move.V * move.DeltaT * sin(M_PI_2 - location.theta));
            std::cout << location.x << ", " << location.y << std::endl;
            painter.setPen(Qt::black);
            painter.drawLine(location.x * scale + centerX, height - (location.y * scale + centerY), x2 * scale + centerX, height - (y2 * scale + centerY));
        }
        else
        {
            // Draw arc
            float R = (move.V / fabs(move.W));

            double x, y, a, alen;

            if(move.W > 0)
            {
                x = ( location.x ) - R + ( R * cos(location.theta) );
                y = ( location.y ) - R - ( R * sin(location.theta) );
                a = AngleUtils::radsToDeg(M_PI - location.theta) * 16;
                alen = AngleUtils::radsToDeg(move.W * move.DeltaT) * 16;
                painter.setPen(Qt::red);
                std::cout << "arc\tw>0\n\tx = " << x + centerX << " y = " << y + centerY << std::endl;
            }
            else
            {
                x = ( location.x ) - R - ( R * cos(location.theta) );
                y = ( location.y ) - R + ( R * sin(location.theta) );
                a = AngleUtils::radsToDeg(-location.theta) * 16;
                alen = AngleUtils::radsToDeg(-move.W * move.DeltaT) * 16;
                painter.setPen(Qt::green);
                std::cout << "arc\tw<0\n\tx = " << (x + centerX) << " y = " << (y + centerY) << std::endl;
            }
            auto pen = painter.pen();
            painter.setPen(Qt::cyan);
            painter.drawEllipse((x * scale) + centerX, (y  * scale) + centerY, scale * 2 * R, scale * 2 * R);
            painter.setPen(pen);
            painter.drawArc((x * scale) + centerX, (y  * scale) + centerY, scale * 2 * R, scale * 2 * R, a, alen);
        }
    }
    std::cout << std::endl;

    painter.end();
}

void PathAdapter::wheelEvent(QWheelEvent *event)
{
    scale += 0.01 * event->delta();
    scale = max(scale, 0.);
    scale = min(scale, 100.);
    this->update();
}
