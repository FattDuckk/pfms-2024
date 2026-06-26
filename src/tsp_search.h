#ifndef TSP_SEARCH_H
#define TSP_SEARCH_H

#include "controllerinterface.h"
#include <cmath>

#include <stack>
#include <queue>
#include <algorithm>
#include <mutex>
#include "geometry_msgs/msg/point.hpp"

/**
 * @brief Class for solving the Traveling Salesman Problem (TSP) using a brute force approach.
 */

class TSP_search
{
public:
  /**
   * @brief Constructs the TSP_search with specified goals, starting point, and controller.
   * @param goals A vector of pointers to the goal points.
   * @param start The index of the starting goal.
   * @param controller A pointer to the controller interface used for calculating odometry and distances between points.
   */
  TSP_search(std::vector<geometry_msgs::msg::Point*> goals, int start, int end);


  /**
   * @brief Solves the TSP using a brute force approach and returns the optimal route as a sequence of goal indices.
   * @return A vector of integers representing the indices of goals in the order they should be visited to minimize the travel distance.
   */
  std::vector<int> brute_force_tsp();

private:
  // Vector of pointers to goal points
  std::vector<geometry_msgs::msg::Point*> goals_;
  
  // Vector storing the indices of goals in the optimal order
  std::vector<int> optimal_order;
  
  // Vector of node indices used in the brute force search
  std::vector<int> nodes;

  // Flag to indicate if the current permutation is a valid solution
  bool OK;

  // Index of the current goal in the route calculation
  unsigned int idx;

  // The total distance for the current permutation
  double distance;

  // The total time for the current permutation
  double time;

  // The current distance traveled in the route calculation
  double current_distance_;

  // Mutex for thread safety in multi-threaded contexts
  std::mutex mtx;

  // Index of the starting goal
  int start_;
  int end_; // Index of the ending/destination goal


  // Minimal distance found for any permutation
  double minimal_distance_;

  double calculate_distance(geometry_msgs::msg::Point* pt1, geometry_msgs::msg::Point* pt2);
};


#endif // TSP_SEARCH_H
