#include <pluginlib/class_list_macros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "d_lite_global_planner.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(d_lite_global_planner::DLiteGlobalPlanner, nav_core::BaseGlobalPlanner);

using namespace std;

// Default Constructor
namespace d_lite_global_planner {

    void print(std::vector<int> const &a) {
        for (int i = 0; i < a.size(); i++) {
            std::cout << a.at(i) << ' ';
        }
        std::cout << std::endl;
    }

    DLiteGlobalPlanner::DLiteGlobalPlanner () {
    }

    DLiteGlobalPlanner::DLiteGlobalPlanner(std::string name,
            std::vector<std::vector<int>> cost_map) {
        initialize(name, cost_map);
    }

    DLiteGlobalPlanner::DLiteGlobalPlanner(std::string name,
            costmap_2d::Costmap2DROS* costmap_ros) {
        // initialize(name, costmap_ros);
    }

    int DLiteGlobalPlanner::heuristicCalculator(int x1, int x2, int y1, int y2) {
        return std::abs(y1 - y2) + std::abs(x1 - x2);
    }

    std::vector<int> DLiteGlobalPlanner::CalculateKey(std::vector<int> s) {
        std::vector<int> key_value;
        int heuristic_distance = heuristicCalculator(this->goal[0], s[0], this->goal[1], s[1]);
        key_value.push_back(std::min(s[2], s[3]) + heuristic_distance + this->new_priority_k);
        key_value.push_back(std::min(s[2], s[3]));
        return key_value;
    }

    void DLiteGlobalPlanner::initialize(std::string name,
            costmap_2d::Costmap2DROS* costmap_ros) {
    }

    void DLiteGlobalPlanner::initialize(std::string name,
            std::vector<std::vector<int>> cost_map) {
        // Initialize priority_k to 0
        this->new_priority_k = 0;

        // Initialize cost_map
        this->cost_map = cost_map;

        // Initialize the goal
        this->goal = this->cost_map.at(this->cost_map.size() - 1);

        // Set all the g-values and rhs-values in the cost map to infinity (highest int in this case)
        for (int i = 0; i < this->cost_map.size(); i++) {
            std::vector<int> &vertex = this->cost_map.at(i);
            vertex[2] = INT_MAX;
            vertex[3] = INT_MAX;
        }
    }

    void DLiteGlobalPlanner::computeShortestPath() {

    }

    bool DLiteGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan) {
        computeShortestPath();
        // After the shortest path from start to goal is found by using D* Lite algorithm (goal to start)
        plan.push_back(start);

        plan.push_back(goal);
        return true;
    }
};