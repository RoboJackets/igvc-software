/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;
using std::vector;

struct pq_comparator {
    bool operator()(std::vector<int> const& a, std::vector<int> const& b) const {

        // sanity checks
        assert(a.size() == 2);
        assert(b.size() == 2);

        // check first component of the two vectors
        if (a[0] != b[0]) {
            return a[0] < b[0];
        }
        return a[1] < b[1];
    }
};

#ifndef SRC_DLITEGLOBALPLANNER_H
#define SRC_DLITEGLOBALPLANNER_H

namespace d_lite_global_planner {
class DLiteGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    DLiteGlobalPlanner();
    DLiteGlobalPlanner(std::string name, std::vector<std::vector<int>> costmap);
    DLiteGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    std::vector<int> CalculateKey(std::vector<int> s);

    int heuristicCalculator(int x1, int x2, int y1, int y2);

    void updateVertex();

    void computeShortestPath();

    void initialize(std::string name, std::vector<std::vector<int>> costmap);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);
private:
    // Cost Map for the algorithm
    costmap_2d::Costmap2DROS* costmap_ros();
    std::vector<std::vector<int> > cost_map;

    // Goal for the algorithm to get to
    std::vector<int> goal;

    // k-value to adjust the priorities when the robot moves from an old location to a new one
    int new_priority_k;

    // PriorityQueue for the vertex cost values
    std::priority_queue<int, std::vector<int>, pq_comparator > priorities;
};
}

#endif // SRC_GLOBALPLANNER_H
