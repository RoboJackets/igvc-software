//
// Created by aaronmao on 12/29/19.
//

#ifndef SRC_FAKE_CONE_H
#define SRC_FAKE_CONE_H


#include <nav_msgs/OccupancyGrid.h>

namespace fake_cone{
    class FakeConeService {
    public:
        void scanAndGenerate(int x, int y);

    private:
        void linearProbe(nav_msgs::OccupancyGrid &localCostMap, int index, std::vector<std::vector<int>> &lines, std::vector<int> line, std::vector<int> visited);
        std::vector<int> nearbyOccupied(nav_msgs::OccupancyGrid &localCostMap, int index);
        std::vector<int> gatherNearby(nav_msgs::OccupancyGrid &localCostMap, int index);
        std::vector<int> filterOccupied(nav_msgs::OccupancyGrid &localCostMap, std::vector<int> input);
    };
}


#endif //SRC_FAKE_CONE_H
