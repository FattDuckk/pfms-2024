#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h> //To use getYaw function from the quaternion of orientation
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "controllerinterface.h"

// Including status from pfms_types.h
#include "pfms_types.h"
using std::placeholders::_1;

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    geometry_msgs::msg::Point location;   //pfms::geometry_msgs::Point location;
    double distance;                      //! distance to goal
    double time;                          //! time to goal
};

/**
 * \brief Shared functionality/base class for platform controllers
 *
 * Platforms need to implement:
 * - Controller::calcNewGoal (and updating GoalStats)
 * - ControllerInterface::reachGoal (and updating PlatformStats)
 * - ControllerInterface::checkOriginToDestination
 * - ControllerInterface::getPlatformType
 * - ControllerInterface::getOdometry (and updating PlatformStats.odo)
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual bool calcNewGoal() = 0;

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
//in A1/A2 was : bool setGoal(pfms::geometry_msgs::Point goal);
void setGoal(const geometry_msgs::msg::Point& msg);  //!< Sets the goal for the platform

void laserCallback(const sensor_msgs::msg::LaserScan& msg);  //!< Callback function for laser scan data

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                    geometry_msgs::msg::Point goal,
                    double& distance,
                    double& time,
                    geometry_msgs::msg::Pose& estimatedGoalPose) = 0;


  //pfms::PlatformType getPlatformType(void);

  bool setTolerance(double tolerance);  //!< Sets the tolerance radius for reaching the goal

  double distanceTravelled(void);  //!< Returns the total distance travelled by the platform

  double timeInMotion(void);  //!< Returns the total time spent in motion by the platform

  double distanceToGoal(void);  //!< Returns the current distance to the goal

  double timeToGoal(void);  //!< Returns the estimated time to reach the goal

  void updateDistanceTravelled();  //!< Updates the total distance travelled by the platform

  void updateTimeInMotion();  //!< Updates the total time spent in motion by the platform

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepte d
   */
  //pfms::nav_msgs::Odometry getOdometry(void);
  geometry_msgs::msg::Pose getOdometry(void);  //!< Returns the current odometry of the platform

  virtual void odoCallback(const nav_msgs::msg::Odometry& msg);  //!< Callback function for odometry data

  // The callback for the service
  // virtual void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res)=0;

protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own pose before calling!
   * @return true if the goal is reached
   */
  bool goalReached();  //!< Checks if the goal has been reached

  /** 
   * @brief Returns a string derived from current status
  */
  std::string getInfoString();  //!< Returns a string derived from the current status

  geometry_msgs::msg::Pose pose_;//!< The current pose of platform
  geometry_msgs::msg::Pose previous_pose_;//!< The previous pose of platform
  geometry_msgs::msg::Pose est_final_pose;
  bool previous_pose_empty;

  std::chrono::_V2::system_clock::time_point time_start;   /**< Time Measurement Start */
  std::chrono::_V2::system_clock::time_point time_stop;    /**< Time Measurement Ends */

  std::mutex poseMtx_;//!< Mutex to protect the pose
  std::mutex velMtx_;//!< Mutex to protect the pose
  std::mutex laserMtx_;//!< Mutex to protect the pose
  

  //stats
  GoalStats goal_;  //!< Information about the goal for the platform
  bool goalSet_;  //!< Flag indicating if a goal has been set

  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

  sensor_msgs::msg::LaserScan laserData;  //!< Laser scan data received by the platform
  
  //Instead of Pipes now we use ROS communication mechanism
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;  //!< Subscription for odometry data
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;  //!< Subscription for goal data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;  //!< Subscription for laser scan data
  
  std::atomic<bool> emergency_stop;  //!< Flag indicating emergency stop
  pfms::PlatformStatus status_;//!< The status of the platform
  pfms::geometry_msgs::Point pfmsGoal; //!< The goal for the platform in pfms format

};

#endif // CONTROLLER_H
