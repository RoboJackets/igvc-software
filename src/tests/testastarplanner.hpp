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
    void newPath(path_t)
    {
//        std::cout << "Path of length " << path.size() << " recieved." << std::endl;
        pathRecieved = true;
    }

private slots:
    void initTest() {
        pathRecieved = false;
    }

    void testNoObstacles() {
        AStarPlanner planner;
        connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
        connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
        connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
        connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

        setStart(RobotPosition(0,0,0));
        setEnd(RobotPosition(10,10,0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        map->push_back(pcl::PointXYZ(100,100,100));
        QBENCHMARK_ONCE {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 1);
        std::cout << "Path of length " << planner.GetPath().size() << " recieved." << std::endl;
//        for(auto pair : planner.GetPath())
//            std::cout << "\t" << pair.second.x << ", " << pair.second.y << "\t" << pair.first.V << ", " << pair.first.W << std::endl;

        path_t path = planner.GetPath();
        QVERIFY2(path[path.size()-1].second.distTo(SearchLocation(10,10,0)) < 2.0, "Path does not really reach the goal.");
    }

    void testWithObstacles() {
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
//        map->push_back(pcl::PointXYZ(4,5,0));
        QBENCHMARK {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 5);
        std::cout << "Path of length " << planner.GetPath().size() << " recieved." << std::endl;
//        for(auto pair : planner.GetPath())
//            std::cout << "\t" << pair.second.x << ", " << pair.second.y << std::endl;
    }

    void testWithObstacles2() {
        AStarPlanner planner;
        connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
        connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
        connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
        connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

        setStart(RobotPosition(0,0,0));
        setEnd(RobotPosition(-10,-10,0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
//        for(int x = -10; x < 10; x++)
//            map->push_back(pcl::PointXYZ(x,5,0));
        map->push_back(pcl::PointXYZ(5,5,0));
        QBENCHMARK {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 5);
        std::cout << "Path of length " << planner.GetPath().size() << " recieved." << std::endl;
//        for(auto pair : planner.GetPath())
//            std::cout << "\t" << pair.second.x << ", " << pair.second.y << std::endl;
    }

    void testWithObstacles3() {
        AStarPlanner planner;
        connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
        connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
        connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
        connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

        setStart(RobotPosition(0,0,0));
        setEnd(RobotPosition(10,10,0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        for(int x = 0; x < 20; x++)
            map->push_back(pcl::PointXYZ(x,5,0));
        map->push_back(pcl::PointXYZ(5,5,0));
        QBENCHMARK {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 5);
        std::cout << "Path of length " << planner.GetPath().size() << " recieved." << std::endl;
//        for(auto pair : planner.GetPath())
//            std::cout << "\t" << pair.second.x << ", " << pair.second.y << std::endl;
    }

    void testUnsolvable() {
        AStarPlanner planner;
        connect(&planner, SIGNAL(OnNewPath(path_t)), this, SLOT(newPath(path_t)));
        connect(this, SIGNAL(setStart(RobotPosition)), &planner, SLOT(OnNewStartPos(RobotPosition)));
        connect(this, SIGNAL(setEnd(RobotPosition)), &planner, SLOT(OnNewGoalPos(RobotPosition)));
        connect(this, SIGNAL(setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), &planner, SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

        setStart(RobotPosition(0,0,0));
        setEnd(RobotPosition(10,10,0));
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>());
        for(double theta = 0; theta < 2*M_PI; theta+=0.1)
            map->push_back(pcl::PointXYZ(cos(theta),sin(theta),0));
        QBENCHMARK {
            setMap(map);
        }
        QTRY_VERIFY_WITH_TIMEOUT(pathRecieved, 5);
//        for(auto pair : planner.GetPath())
//            std::cout << "\t" << pair.second.x << ", " << pair.second.y << std::endl;
        QVERIFY2(planner.GetPath().size() == 0, "Unsolvable problem generated a non-zero length path.");
    }
};

#endif // TESTASTARPLANNER_HPP
