#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "audi.h"

namespace ackerman
{
  /**
   * @brief Motion state of an Ackerman-steered vehicle
   */
  enum State
  {
    Accelerate, /*!< Vehicle is accelerating */
    Decelerate, /*!< Vehicle is decelerating */
    Stopped     /*!< Vehicle is stopped */
  };
}

//! Ackerman Audi platform controller
class Ackerman : public Controller, public rclcpp::Node
{
public:
  /**
   * @brief Constructor for Ackerman class
   */
  Ackerman();

  /**
   * @brief Destructor for Ackerman class
   */
  ~Ackerman();

  /**
   * @brief Check if the goal has been reached
   * @return True if the goal has been reached, false otherwise
   */
  bool reachGoal(void);

  /**
   * @brief Calculates the whether the ackerman need to starts braking or not.
   * @return true if ackerman has no unreachable goals.
   */
  bool calcNewGoal(void);

  /**
   * @brief Check the distance and time from origin to destination
   * @param origin The origin pose
   * @param goal The goal point
   * @param distance The calculated distance from origin to destination
   * @param time The calculated time from origin to destination
   * @param estimatedGoalPose The estimated goal pose
   * @return True if the check is successful, false otherwise
   */
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal,
                                double &distance, double &time,
                                geometry_msgs::msg::Pose &estimatedGoalPose);

  // The callback for the service
  // void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // void setGoal(const geometry_msgs::msg::Point& msg);
  // void odoCallback(const nav_msgs::msg::Odometry& msg);

  /**
   * @brief Callback function for velocity messages
   * @param msg The received velocity message
   */
  void velocityCallback(const geometry_msgs::msg::TwistStamped &msg);

  /**
   * @brief Update brake information
   * @return True if the update is successful, false otherwise
   */
  bool updateBrakeInfo();

private:
  /**
   * @brief Send command to ackerman for control
   * @param brake Brake value
   * @param steering Steering
   * @param throttle Throttle
   */
  void sendAckCmd(double brake, double steering, double throttle);

  /**
   * @brief Callback function for odometry messages
   * @param msg The received odometry message
   */
  void odoCallback(const nav_msgs::msg::Odometry &msg) override;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_; //!< ROS2 Subscriber for velocity
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubBrake_;    //!< ROS2 Publisher for brake
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubSteering_; //!< ROS2 Publisher for steering
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubThrottle_; //!< ROS2 Publisher for throttle
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubEmergencyStop_; //!< ROS2 Publisher for emergency stop
  rclcpp::TimerBase::SharedPtr timer_;                               //!< Timer object for periodically calling the reachGoal method.

  std_msgs::msg::Float64 brake_msg;    //!< Brake message
  std_msgs::msg::Float64 steering_msg; //!< Steering message
  std_msgs::msg::Float64 throttle_msg; //!< Throttle message

  pfms::nav_msgs::Odometry poseToOdo(geometry_msgs::msg::Pose poseConv_); //!< Convert pose to odometry; nav_msgs::Odometry to pfms::nav_msgs::Odometry
  pfms::nav_msgs::Odometry odo_;                                          //!< Odometry
  pfms::nav_msgs::Odometry currentOdo_;                                   //!< Current odometry
  pfms::nav_msgs::Odometry estimatedGoalOdo_;                             //!< Estimated goal odometry

  geometry_msgs::msg::Pose odoToPose(pfms::nav_msgs::Odometry odoConv_);  //!< Convert odometry to pose; pfms::nav_msgs::Odometry to geometry_msgs::msg::Pose

  Audi audi;                     //!< Audi object
  ackerman::State ackermanState; //!< Ackerman state

  const double TARGET_SPEED; //!< The target speed for the ackerman
  bool audi_reachable;       //!< Flag for the ackerman reachable goals
  bool goal_reachable;       //!< Flag for the ackerman unreachable

  double brake_;    //!< Brake value
  double throttle_; //!< Throttle value
  double steering_; //!< Steering value

  double distance;    //!< Distance travelled
  double velocity;    //!< Velocity of the audi
  double work;        //!< Work for brake calculation
  double mass;        //!< Mass of the Audi
  double brake_force; //!< Default brake force
};

#endif // ACKERMAN_H
