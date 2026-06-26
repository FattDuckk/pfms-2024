#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <condition_variable>
#include "tsp_search.h"
#include "dataEvaluation.h"
// #include "marker_visualisers.h"

namespace mission
{
  enum roadState
  { /*!< Motion state of an Ackerman-steered vehicle */
    DRONE_PATH, /*!< Drone Path */
    ROAD,        /*!< Road */
    DANGER,     /*!< Danger Point */
    WAYPOINT  /*!< Intermediate Waypoint */
  };

  enum mode
  { /*!< Motion state of an Ackerman-steered vehicle */
    BASIC,
    ADVANCED /*!< Vehicle is accelerating */
  };

}

/**
 * @brief Mission class for controlling the mission.
 */
class Mission : public rclcpp::Node
{
public:
  /**
   * @brief Default constructor for Mission class.
   */
  Mission();

  /**
   * @brief Destructor for Mission class.
   */
  ~Mission();

  /**
   * @brief Sets the goal for the drone.
   * @param goal The goal position for the drone.
   */
  void cmd_drone_goal(geometry_msgs::msg::Point goal);

  /**
   * @brief Sets the goal for the orange.
   * @param goal The goal position for the orange.
   */
  void cmd_orange_goal(geometry_msgs::msg::Point goal);

  mission::roadState road_state_; //!< Current road state
  mission::mode mode_;           //!< Current mode

private:
  /**
   * @brief Function that runs continuously in a separate thread.
   */
  void run();

  /**
   * @brief Function that updates the progress of the mission.
   *        Triggered by a timer.
   */
  void progress();

  /**
   * @brief Calculates the distance between two points.
   * @param odo The odometry data.
   * @param pt The point to calculate the distance to.
   * @return The distance between the odometry and the point.
   */
  double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);

  /**
   * @brief Callback function for receiving goal pose messages.
   * @param msg The received goal pose message.
   */
  void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

  /**
   * @brief Callback function for receiving drone odometry messages.
   * @param msg The received drone odometry message.
   */
  void odomCallback_drone(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

  /**
   * @brief Callback function for receiving orange odometry messages.
   * @param msg The received orange odometry message.
   */
  void odomCallback_orange(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

  /**
   * @brief Callback function for receiving sonar range messages.
   * @param msg The received sonar range message.
   */
  void sonarCallback_(const std::shared_ptr<sensor_msgs::msg::Range> msg);

  /**
   * @brief Generates intermediate waypoints between two points.
   * @param start The starting pose_.
   * @param goal The goal point.
   * @param interval The interval between waypoints.
   * @return The generated waypoints.
   */
  std::vector<geometry_msgs::msg::Point> generateIntermediateWaypoints(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, double interval);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_goal_; //!< Subscriber for 2D Goal Pose
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_drone_odo_;  //!< Subscriber for drone odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_orange_odo_; //!< Subscriber for orange odometry
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_;      //!< Subscriber for drone sonar

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_drone_goal_;        //!< Publisher for drone goals
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_orange_goal_;       //!< Publisher for orange goals
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_visual_; //!< Publisher for visualization markers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_waypoint;       //!< Publisher for goals
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_danger;         //!< Publisher for goals

  /**
   * @brief Callback function for controlling the mission.
   * @param req The request for controlling the mission.
   * @param res The response for controlling the mission.
   */
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv1_mission_;

  std::thread *thread_;                //!< Pointer to the thread object
  rclcpp::TimerBase::SharedPtr timer_; //!< Pointer to the timer object, used to run a function at regular intervals

  /**
   * @brief Evaluates the gradient between two heights.
   * @param initial_height The initial height.
   * @param final_height The final height.
   * @param distance The distance between the heights.
   * @param goal The goal position.
   */
  void evaluateGradient(float initial_height, float final_height, float distance, geometry_msgs::msg::Point goal);

  /**
   * @brief Calculates the height.
   * @return The calculated height.
   */
  float calHeight();

  void visualizeWaypoints();

  /**
   * @brief Produces a marker for visualization.
   * @param pt The point for the marker.
   * @param state The road state for the marker.
   * @return The produced marker.
   */
  visualization_msgs::msg::Marker produceMarker(geometry_msgs::msg::Point pt, mission::roadState state);

  visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array
  std::mutex mtxOdo_drone_;                          //!< Mutex for drone odometry
  std::mutex mtxOdo_orange_;                         //!< Mutex for orange odometry
  std::mutex mtxSonar;                               //!< Mutex for sonar
  std::mutex mtxSonarData;                           //!< Mutex for sonar data
  std::mutex mtxOrangeGoals;                         //!< Mutex for orange goals
  std::mutex mtxDroneGoals;                          //!< Mutex for drone goals

  nav_msgs::msg::Odometry drone_pose_;  //!< Storage for drone odometry
  nav_msgs::msg::Odometry orange_pose_; //!< Storage for orange odometry

  std::mutex mtxDroneGoals_;  //!< Mutex for drone goals and initial distance to goal
  std::mutex mtxOrangeGoals_; //!< Mutex for orange goals and initial distance to goal
  std::mutex cond_mtx;        //!< Mutex for condition variable
  std::condition_variable cond_var; //!< Condition variable
  std::vector<geometry_msgs::msg::Pose> pt_vec; //!< Vector of poses
  std::deque<geometry_msgs::msg::Point> drone_goals_;  //!< Storage for drone goals
  std::deque<geometry_msgs::msg::Point> orange_goals_; //!< Storage for orange goals
  std::deque<geometry_msgs::msg::Point> orange_unordered_goals_; //!< Storage for orange goals
  std::deque<geometry_msgs::msg::Point> orange_danger_point; //!< Storage for orange goals
  sensor_msgs::msg::Range sonarData; //!< Sonar range data

  float groundRange_;    //!< Ground range
  float avgSonar;        //!< Average sonar range
  float sumSonar;        //!< Sum of sonar range
  float gndHeight;       //!< Ground height
  float current_gndHeight; //!< Current ground height
  float dis_drone_goals_; //!< Distance to drone goals
  float gradient;        //!< Gradient
  float sonarRange_;     //!< Sonar range
  float min_Range_;      //!< Minimum range
  float max_Range_;      //!< Maximum range
  float previous_gndHeight; //!< Previous ground height

  double init_dist_to_goal_drone_; //!< Initial distance to goal
  double init_dist_to_goal_orange_; //!< Initial distance to goal
  double tolerance_;         //!< Tolerance

  unsigned int ct_; //!< Marker Count
  unsigned int sonarCount;
  unsigned int progressDrone; //!< Progress of the drone
  unsigned int progressOrange; //!< Progress of the orange

  std::atomic<bool> newGoals;     //!< Indicates if new goals are available
  std::atomic<bool> ongoing_goal; //!< Indicates if a goal is ongoing
  std::atomic<bool> drone_chasing_;  //!< Indicates if the drone is chasing a goal
  std::atomic<bool> orange_chasing_; //!< Indicates if the orange is chasing a goal
  std::atomic<bool> mission_status;   //!< Mission status
  std::atomic<bool> goalStatus;      //!< Goal status

  bool eval_done;          //!< Indicates if evaluation is done
  bool advanced_mode_;     //!< Indicates if advanced mode is enabled
  DataEvaluator dataEval; //!< Data evaluation object
  
};
