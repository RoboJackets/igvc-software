#include "pathadapter.h"
#include "ui_pathadapter.h"
#include <QPainter>
#include "intelligence/pathplanning/pathplanner.hpp"
#include <math.h>
#include "intelligence/pathplanning/astarplanner.h"
#include "common/utils/AngleUtils.h"
#include <math.h>
#include <common/utils/AngleUtils.h>

PathAdapter::PathAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PathAdapter)
{
    ui->setupUi(this);

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

    int startX = painter.window().width()/2;
    int startY = painter.window().height()/2;
    int x = startX;
    int y = startY;
    QPainterPath dpath;
    dpath.moveTo(startX,startY);
    for(int i=0;i<path.size()-1;i++)
    {
        std::pair<SearchMove, SearchLocation> p1 = path[i];
        std::pair<SearchMove, SearchLocation> p2 = path[i+1];

        int height = abs(round((p1.second.y - p2.second.y)*30));
        int width = abs(round((p1.second.x - p2.second.x)*30));
        //dpath.lineTo(p1.second.x * 10+ startX, p1.second.y * 10+ startY);

        if(p1.first.W==0.0)
        {
            painter.drawLine(p1.second.x * 10 + startX,p1.second.y * 10 + startY,p2.second.x * 10 + startX,p2.second.y * 10 + startY);
        }
        else
        {
            painter.drawArc(round(p1.second.x*30) + startX, round(p1.second.y*30) + startY, width, height, round(AngleUtils::radsToDeg((M_PI/2)-p1.second.theta)*16), round(AngleUtils::radsToDeg((M_PI/2)-(p2.second.theta-p1.second.theta))*16));

        }

        std::cout << "x " << round(p1.second.x*30) + startX << " y " << round(p1.second.y*30) + startY << " w " << width << " h " << height << " a " << round(AngleUtils::radsToDeg((M_PI/2)-p1.second.theta)) << " alen " << round(AngleUtils::radsToDeg((M_PI/2)-(p2.second.theta-p1.second.theta))) << std::endl;
   }
    //painter.drawArc(0,0,painter.window().width(),painter.window().height(),0,5760*(16 * 360));


    //painter.drawPath(dpath);
    painter.end();
}
