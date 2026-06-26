#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

//We include messages types for quadcopter
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

//! Quadcopter platform controller
class Quadcopter: public Controller, public rclcpp::Node
{
public:
  /**
   * @brief Constructor for Quadcopter class.
   */
  Quadcopter(); 

  /**
   * @brief Destructor for Quadcopter class.
   */
  ~Quadcopter();

  /**
   * @brief Checks if the quadcopter has reached its goal.
   * @return True if the quadcopter has reached the goal, false otherwise.
   */
  bool reachGoal(void);

  /**
   * @brief Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);

  /**
   * @brief Checks the distance and time from the origin to the destination.
   * @param origin The origin pose.
   * @param goal The goal point.
   * @param distance The calculated distance from origin to destination.
   * @param time The calculated time from origin to destination.
   * @param estimatedGoalPose The estimated goal pose.
   * @return True if the calculation is successful, false otherwise.
   */
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal,
                                 double& distance, double& time,
                                 geometry_msgs::msg::Pose& estimatedGoalPose);

  /**
   * @brief The callback function for the control service.
   * @param req The request for the control service.
   * @param res The response for the control service.
   */
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // void setGoal(const geometry_msgs::msg::Point& msg);  
  // void odoCallback(const nav_msgs::msg::Odometry& msg);

private:

  /** 
   * @brief Sends command to the quadcopter for control.
   * @param turn_l_r Turn left/right.
   * @param move_l_r Move left/right.
   * @param move_u_d Move up/down.
   * @param move_f_b Move forward/backward.
   */
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  /**
   * @brief Sends takeoff command to the quadcopter.
   */
  void sendTakeOff(void);

  /**
   * @brief Sends landing command to the quadcopter.
   */
  void sendLanding(void);

  //! Angle required for the quadcopter to have a straight shot at the goal.
  double target_angle_ = 0;

  bool liftoff_; //!< Flag for the quadcopter liftoff.

  const double TARGET_SPEED; //!< The target speed for the quadcopter.
  const double TARGET_HEIGHT_TOLERANCE; //!< The tolerance for the target height.

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_; //!< Publisher for the command velocity.
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_; //!< Publisher for the takeoff command.
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLanding_; //!< Publisher for the landing command.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubEmergencyStop_; //!< Publisher for the emergency command.
  rclcpp::TimerBase::SharedPtr timer_; //!< Timer object for periodically calling the reachGoal method.

};

#endif // QUADCOPTER_H
