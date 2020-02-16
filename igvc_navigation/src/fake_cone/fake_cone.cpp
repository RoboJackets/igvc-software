//
// Created by aaronmao on 12/29/19.
//

#include <ros/ros.h>
#include "fake_cone.h"
#include <nav_msgs/OccupancyGrid.h>


int OCCUPY_THRESHOLD = 70;
double COEFFICIENT_THRESHOLD = 0.8;

namespace fake_cone
{
    bool FakeConeService::equals(geometry_msgs::Point a, geometry_msgs::Point b) {
        return a.x == b.x && a.y == b.y;
    }

    bool FakeConeService::contains(std::vector<geometry_msgs::Point> &array, geometry_msgs::Point point) {
        bool result = true;
        for (geometry_msgs::Point element : array) {
            if (equals(element, point)) {
                result = false;
                break;
            }
        }

        return result;
    }

    geometry_msgs::Point FakeConeService::convert1DIndexTo2D(nav_msgs::OccupancyGrid &localMap, int index) {
        geometry_msgs::Point point;

        point.x = index % localMap.info.width;
        point.y = index / localMap.info.width;

        return point;
    }

    int FakeConeService::convert2DIndexTo1D(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point) {
        int result = point.y * localCostMap.info.width + point.x;
    }

    std::vector<geometry_msgs::Point> FakeConeService::linearProbe(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point) {
        std::vector<geometry_msgs::Point> possibleLine;
        std::vector<geometry_msgs::Point> visited;
        std::vector<geometry_msgs::Point> queue;

        //Initialize the queue
        for (geometry_msgs::Point element : nearbyOccupied(localCostMap, point)) {
            if (localCostMap.data[convert2DIndexTo1D(localCostMap, element)] > OCCUPY_THRESHOLD) {
                queue.push_back(element);
            }
        }

        while (!queue.empty()) {
            geometry_msgs::Point element = queue.back();
            visited.push_back(element);
            possibleLine.push_back(element);
            for (geometry_msgs::Point nearbyPoint : nearbyOccupied(localCostMap, element)) {
                if (!contains(visited, nearbyPoint)) {
                    queue.push_back(nearbyPoint);
                }
            }
            queue.pop_back();
        }

        return possibleLine;
    }

    std::vector<geometry_msgs::Point> FakeConeService::nearbyOccupied(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point) {
        return filterOccupied(localCostMap, gatherNearby(localCostMap, point));
    }

    std::vector<geometry_msgs::Point> FakeConeService::filterOccupied(nav_msgs::OccupancyGrid &localCostMap, std::vector<geometry_msgs::Point> input) {

        std::vector<geometry_msgs::Point> occupiedList;
        for (geometry_msgs::Point point : input) {
            int index = point.x + point.y * localCostMap.info.width;
            if (localCostMap.data[index] > OCCUPY_THRESHOLD) {
                occupiedList.push_back(point);
            }
        }

        return occupiedList;
    }

    std::vector<geometry_msgs::Point> FakeConeService::gatherNearby(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point point) {

        geometry_msgs::Point temp;
        std::vector<geometry_msgs::Point> nearbyPoints;

        //this is on the left edge
        if ((int)point.x % localCostMap.info.width == 0) {
            temp = geometry_msgs::Point{point};
            temp.x = temp.x + 1;
            nearbyPoints.push_back(temp);
            temp.y = temp.y + 1;
            nearbyPoints.push_back(temp);
            temp.y = temp.y - 2;
            nearbyPoints.push_back(temp);
        }
        //this is on the right edge
        else if ((int) point.x % localCostMap.info.width == localCostMap.info.width - 1) {
            temp = geometry_msgs::Point{point};
            temp.x = temp.x - 1;
            nearbyPoints.push_back(temp);
            temp.y = temp.y + 1;
            nearbyPoints.push_back(temp);
            temp.y = temp.y - 2;
            nearbyPoints.push_back(temp);
        }

        temp = geometry_msgs::Point{point};
        temp.y = temp.y + 1;
        nearbyPoints.push_back(temp);
        temp.y = temp.y - 2;
        nearbyPoints.push_back(temp);

        return nearbyPoints;
    }

    std::vector<geometry_msgs::Point> FakeConeService::connectEndpoints(geometry_msgs::Point left, geometry_msgs::Point right) {
        std::vector<geometry_msgs::Point> connectingPoint;
        connectingPoint.push_back(left);
        connectingPoint.push_back(right);

        int slope_new = 2 * (right.y - left.y);
        int slope_error = slope_new - (right.x - left.x);

        for (int x = left.x, y = left.y; x <= right.x; x++) {
            geometry_msgs::Point temp;
            temp.x = x;
            temp.y = y;
            connectingPoint.push_back(temp);

            if (slope_error >= 0) {
                y++;
                slope_error -= 2 * (right.x - left.x);
            }
        }
    }

    double FakeConeService::calculateRCoefficient(nav_msgs::OccupancyGrid &localMap,  std::vector<geometry_msgs::Point> linePoints) {

        double sigmaX = 0;
        double sigmaY = 0;
        double sigmaXY = 0;
        double sigmaXsqr = 0;
        double sigmaYsqr = 0;
        int size = linePoints.size();

        for(geometry_msgs::Point linePoint : linePoints) {
            sigmaX += linePoint.x;
            sigmaY += linePoint.y;
            sigmaXY += (linePoint.x * linePoint.y);
            sigmaXsqr += linePoint.x * linePoint.x;
            sigmaYsqr += linePoint.y * linePoint.y;
        }

        double slope = ((size * sigmaXY) - (sigmaX * sigmaY)) / (size * sigmaXsqr - sigmaX * sigmaX);
        double intercept = (sigmaY - slope * sigmaX) / size;
        double rCoefficient = (sigmaXY - sigmaX * sigmaY / size)
                              / sqrt((sigmaXsqr - (sigmaX * sigmaX) / size)
                                     * (sigmaYsqr - (sigmaY * sigmaY) / size));

        return rCoefficient;
    }

    /**
     * to_left means that the algorithm will only search to the left of the robot
     */
    geometry_msgs::Point FakeConeService::findLine(nav_msgs::OccupancyGrid &localCostMap, geometry_msgs::Point current_pos, bool to_left) {
        std::vector<geometry_msgs::Point> visited;
        std::vector<geometry_msgs::Point> queue;

        //initialize the queue
        visited.push_back(current_pos);
        for(geometry_msgs::Point element : gatherNearby(localCostMap, current_pos)) {
                if ((to_left && element.x < current_pos.x) || (!to_left && element.x > current_pos.x)) {
                    queue.push_back(element);
                }
        }
        
        while (!queue.empty()) {
            geometry_msgs::Point head = queue.back();
            visited.push_back(head);
            int headOccupancyProb = localCostMap.data[convert2DIndexTo1D(localCostMap, head)];
            //An occupied point is found on the gridmap
            if (headOccupancyProb > OCCUPY_THRESHOLD) {
                return head;
            }

            for (geometry_msgs::Point element : gatherNearby(localCostMap, head)) {
                if (((to_left && element.x < current_pos.x) || (!to_left && element.x > current_pos.x)) && !contains(visited, head)) {
                    queue.push_back(element);
                }
            }
        }
    }

    geometry_msgs::Point FakeConeService::findEndpoint(std::vector<geometry_msgs::Point> line, geometry_msgs::Point point_on_line, geometry_msgs::Point current_pos) {
        
        int curr_y_pos = current_pos.y;
        geometry_msgs::Point lowest_point = point_on_line;
        
        for (geometry_msgs::Point element : line) {
            if (element.y < lowest_point.y) {
                lowest_point = element;
            }
        }

        return lowest_point;
    }
}