#include "tsp_search.h"

TSP_search::TSP_search(std::vector<geometry_msgs::msg::Point*> goals, int start, int end)
:goals_(goals), start_(start), end_(end)
{
    optimal_order.clear();
    minimal_distance_ = 9999;
}

std::vector<int> TSP_search::brute_force_tsp() {
    std::lock_guard<std::mutex> lock(mtx);
    optimal_order.clear();
    
    std::vector<int> nodes(goals_.size());
    std::iota(nodes.begin(), nodes.end(), 0);

    do {
        //If the first node is not equal to start or last node is not equal to end, skip this iteration
        if (nodes.front() != start_ || nodes.back() != end_) continue;

        current_distance_ = 0.0;
        bool OK = true;

        for (size_t i = 1; i < nodes.size(); ++i) {
            unsigned int node1 = nodes.at(i - 1);
            unsigned int node2 = nodes.at(i);
            double distance = calculate_distance(goals_.at(node1), goals_.at(node2));
            current_distance_ += distance;

            if (current_distance_ >= minimal_distance_) {
                OK = false;
                break;
            }
        }

        if (OK && current_distance_ < minimal_distance_) {
            minimal_distance_ = current_distance_;
            optimal_order = nodes;
        }
    } while (std::next_permutation(nodes.begin(), nodes.end()));

    return optimal_order;
}

double TSP_search::calculate_distance(geometry_msgs::msg::Point* pt1, geometry_msgs::msg::Point* pt2) {
    return std::sqrt(std::pow(pt1->x - pt2->x, 2) + std::pow(pt1->y - pt2->y, 2) + std::pow(pt1->z - pt2->z, 2));
}