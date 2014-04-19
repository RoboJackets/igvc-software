#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <QObject>
#include <pcl/common/common.h>
#include <vector>
#include "searchlocation.h"
#include "searchmove.h"
#include <common/datastructures/robotposition.hpp>

typedef std::vector<std::pair<SearchMove, SearchLocation>> path_t;

class PathPlanner : public QObject {
    Q_OBJECT
public:
    PathPlanner() { }
    virtual ~PathPlanner() { }

    virtual path_t GetPath() = 0;

signals:
    void OnNewPath(path_t path);

public slots:
    virtual void OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr) { }
    virtual void OnNewStartPos(RobotPosition) { }
    virtual void OnNewGoalPos(RobotPosition) { }
};

#endif // PATHPLANNER_HPP
