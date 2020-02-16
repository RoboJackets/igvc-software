//
// Created by aaronmao on 12/29/19.
//

#ifndef SRC_FAKE_CONE_H
#define SRC_FAKE_CONE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

namespace fake_cone{
    class FakeConeService{
    public:
        double calculateRCoefficient(nav_msgs::OccupancyGrid &localMap,  std::vector<geometry_msgs::Point> linePoints);
        std::vector<geometry_msgs::Point> linearProbe(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point);
        geometry_msgs::Point findLine(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point current_pos, bool to_left);
        geometry_msgs::Point findEndpoint(std::vector<geometry_msgs::Point> line, geometry_msgs::Point point_on_line, geometry_msgs::Point current_pos);
        std::vector<geometry_msgs::Point> connectEndpoints(geometry_msgs::Point left, geometry_msgs::Point right);

    private:
        bool equals(geometry_msgs::Point a, geometry_msgs::Point b);
        bool contains(std::vector<geometry_msgs::Point> &array, geometry_msgs::Point point);
        std::vector<geometry_msgs::Point> nearbyOccupied(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point);
        std::vector<geometry_msgs::Point> gatherNearby(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point);
        std::vector<geometry_msgs::Point> filterOccupied(nav_msgs::OccupancyGrid &localCostMap, std::vector<geometry_msgs::Point> input);
        geometry_msgs::Point convert1DIndexTo2D(nav_msgs::OccupancyGrid &localMap, int index);
        int convert2DIndexTo1D(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point);

    protected:
    };

}


#endif //SRC_FAKE_CONE_H
