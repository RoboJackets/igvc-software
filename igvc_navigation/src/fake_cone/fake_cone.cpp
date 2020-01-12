//
// Created by aaronmao on 12/29/19.
//

#include <ros/ros.h>
#include "fake_cone.h"
#include <nav_msgs/OccupancyGrid.h>

ros::NodeHandle nh;
ros::Publisher fake_cone_dubug;

int OCCUPY_THRESHOLD = 70;

namespace fake_cone
{
    void localCostmapCallback(nav_msgs::OccupancyGrid &localCostMap) {
        //float mapResolution = localCostMap.info.resolution;
        int mapSize = localCostMap.info.height * localCostMap.info.width;

        //map size 400*400
        for (int grid = 0; grid < mapSize; grid++) {
            if (localCostMap.data[grid] > OCCUPY_THRESHOLD) {

            }
        }
    }

    void FakeConeService::linearProbe(nav_msgs::OccupancyGrid &localCostMap, int index, std::vector<std::vector<int>> &lines, std::vector<int> line, std::vector<int> visited) {

        if(nearbyOccupied(localCostMap, index).empty()) {
            lines.push_back(line);
        } else {
            std::vector<int> nearby = nearbyOccupied(localCostMap, index);
            line.push_back(index);

             for (int i = 0; i < nearby.size(); i++) {
                 for (int j = 0; j < nearby.size(); j++) {
                     if (i == j) {
                         linearProbe(localCostMap, nearby.at(j), lines, line, visited);
                     } else {

                     }
                 }
             }
        }
    }

    std::vector<int> FakeConeService::nearbyOccupied(nav_msgs::OccupancyGrid &localCostMap, int index) {
        return filterOccupied(localCostMap, gatherNearby(localCostMap, index));
    }

    std::vector<int> FakeConeService::filterOccupied(nav_msgs::OccupancyGrid &localCostMap, std::vector<int> input) {

        std::vector<int> result;
        while (!input.empty()) {
            int temp = input.back();
            if (localCostMap.data[temp] > OCCUPY_THRESHOLD) {
                result.push_back(temp);
            }
            input.pop_back();
        }

        return result;
    }

    std::vector<int> FakeConeService::gatherNearby(nav_msgs::OccupancyGrid &localCostMap, int index) {
        int mapWidth = localCostMap.info.width;

        std::vector<int> nearby;

        nearby.push_back(index - mapWidth);
        nearby.push_back(index + mapWidth);

        //If the element is at the start of the row
        if (index % mapWidth == 1) {
            nearby.push_back(index + 1);
            nearby.push_back(index - mapWidth + 1);
            nearby.push_back(index + mapWidth + 1);
        }
        //If the element is at the end of the row
        else if (index % mapWidth == mapWidth - 1) {
            nearby.push_back(index - 1);
            nearby.push_back(index - mapWidth - 1);
            nearby.push_back(index + mapWidth -1);
        }

        return nearby;
    }

    void FakeConeService::scanAndGenerate(int x, int y)
    {
        //Scan the surrounding costmap and then linearly probe for an endpoint
        ros::Subscriber localCostmapSub = nh.subscribe("/move_base_flex/local_costmap/costmap", 0, localCostmapCallback);
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "fake_cone_service");

        fake_cone_dubug = nh.advertise<std::string>("fake_cone/Debug", 1);
    }
}