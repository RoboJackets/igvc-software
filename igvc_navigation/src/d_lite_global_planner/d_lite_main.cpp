//
// Created by robojackets on 1/22/20.
//
#include "d_lite_global_planner.h"

void print(std::vector<int> const &a) {
    for (int i = 0; i < a.size(); i++) {
        std::cout << a.at(i) << ' ';
    }
    std::cout << std::endl;
}

int main() {
    /**
     * Generate a cost map of vertices
     * Every vertex has X, Y coordinates and rhs, g values
    */
    std::vector<std::vector<int> > cost_map;
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            // New Vertex
            std::vector<int> new_vertex = {x, y, 0, 0};
            cost_map.push_back(new_vertex);
        }
    }

    // Print out the contents in cost map as a test
    for (int i = 0; i < cost_map.size(); i++) {
        print(cost_map.at(i));
    }

    // Initialize the D* Lite Global Planner with the 2D vector map
    d_lite_global_planner::DLiteGlobalPlanner d("Sammy", cost_map);

    for (int i = 0; i < cost_map.size(); i++) {
        print(cost_map.at(i));
    }

}