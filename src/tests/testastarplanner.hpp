#ifndef TESTASTARPLANNER_HPP
#define TESTASTARPLANNER_HPP

#include <QtTest>
#include <iostream>
#include <intelligence/pathplanning/astarplanner.h>

class TestAStarPlanner : public QObject {
    Q_OBJECT
private:
    bool pathRecieved;
signals:
    void setStart(RobotPosition pos);
    void setEnd(RobotPosition pos);
    void setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map);
public slots:
    void newPath(path_t path)
    {
        std::cout << "Path of length " << path.size() << " recieved." << std::endl;
        pathRecieved = true;
    }

private slots:
    void initTest() {
        pathRecieved = false;
    }

    void a() {
        /*IGVCSearchProblem problem;
        problem.getResult(SearchLocation(0,0,0), SearchMove(1,0));*/
        SearchLocation a(0,0,0);
        SearchLocation b(1,1,0);
        std::cout << ( a == b ) << std::endl;
    }

    void test() {
        AStarPlanner planner;
        connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
        connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
        connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
        connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

        setStart(RobotPosition(0,0,0));
        setEnd(RobotPosition(10,10,0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        map->push_back(pcl::PointXYZ(100,100,100));
        QBENCHMARK {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 10000);
    }
};

#endif // TESTASTARPLANNER_HPP
