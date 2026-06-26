#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;
using namespace std::chrono_literals;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Initialisation
//!
//! What do we need to subscribe to and publish?

Quadcopter::Quadcopter() :
    Node("quadcopter_control"),
    liftoff_(false),TARGET_SPEED(0.8),
    TARGET_HEIGHT_TOLERANCE(0.2)
{
    tolerance_=0.5;//We set tolerance to be default of 0.5

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone/gt_odom", 1000, std::bind(&Quadcopter::odoCallback,this,_1));
    sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 1000, std::bind(&Quadcopter::setGoal,this,_1));

    pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel",3);  
    pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("/drone/takeoff",3);  
    pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("/drone/land",3);  
    pubEmergencyStop_ = this->create_publisher<std_msgs::msg::Bool>("/drone/emergency_stop",3);

    // srv1_ = this->create_service<std_srvs::srv::SetBool>(
    //     "/drone_reach_goal", std::bind(&Quadcopter::control,this,std::placeholders::_1, std::placeholders::_2)); 

    timer_ = this->create_wall_timer(
    100ms, std::bind(&Quadcopter::reachGoal, this));


};

Quadcopter::~Quadcopter(){

    timer_->cancel(); // Cancel the timer. Avoid the callback to be called after the object is destroyed... I think...
    RCLCPP_INFO(get_logger(), "Quadcopter node shutting down. BEEP, BOOP.");
}


bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin, 
                            geometry_msgs::msg::Point goal,
                            double& distance, double& time,
                            geometry_msgs::msg::Pose& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    //estimatedGoalPose.yaw = origin.yaw; - How do we deal with yaw to quaternion?

    return true;
}

bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    if (!checkOriginToDestination(pose_, goal_.location, goal_.distance, goal_.time, est_final_pose))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - pose_.position.x;
    double dy = goal_.location.y - pose_.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}


void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {

    if(!liftoff_){
        std_msgs::msg::Empty msg = std_msgs::msg::Empty();
        pubTakeOff_->publish(msg);
    }
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    //OR we can do auto msg = geometry_msgs::msg::Twist();
    msg.linear.x= move_f_b;
    msg.linear.y= move_l_r;
    msg.linear.z= move_u_d;
    msg.angular.z = turn_l_r;
    pubCmdVel_->publish(msg);
}


void Quadcopter::sendTakeOff(void) {
    std_msgs::msg::Empty msg;
    pubTakeOff_->publish(msg);
}


void Quadcopter::sendLanding(void) {
    std_msgs::msg::Empty msg;
    pubLanding_->publish(msg);
    status_=pfms::PlatformStatus::IDLE;
}


bool Quadcopter::reachGoal(void) {

    std::cout<<"In reachGoal"<<std::endl;
   
    if (!liftoff_){
        sendTakeOff();
        liftoff_=true;
    }

    switch(status_){
        case pfms::PlatformStatus::IDLE:
            sendCmd(0, 0, 2-pose_.position.z, 0);
            if(goalSet_){
                status_=pfms::PlatformStatus::RUNNING;
            }
            return false;

        case pfms::PlatformStatus::TAKEOFF:
            sendTakeOff();
            if(goalSet_){
                status_=pfms::PlatformStatus::RUNNING;
                //  time_start=std::chrono::high_resolution_clock::now();
            }
            else{
                status_=pfms::PlatformStatus::IDLE;
            }
            return true;

        case pfms::PlatformStatus::LANDING:
            sendLanding();
            status_=pfms::PlatformStatus::IDLE;
            return true;
        case pfms::PlatformStatus::RUNNING:
            break;
    }

    if  (!goalSet_){
        // sendCmd(0, 0, 2-pose_.position.z, 0);
        return false;
    }

    for (auto laserRange : laserData.ranges)
    {

        if (pose_.position.z>1.9){
            if ((laserRange < laserData.range_max) && (laserRange > 0.15) &&
                !std::isnan(laserRange) && std::isfinite(laserRange) && (laserRange < 0.3))
            {
                emergency_stop = true;
                
                std_msgs::msg::Bool emergency_stop_msg;
                emergency_stop_msg.set__data(true);
                pubEmergencyStop_->publish(emergency_stop_msg);
                RCLCPP_INFO_STREAM(get_logger(), "🚨 Alert! Object detected at " << laserRange << " metres");
                break;
            }
        }
    }

    time_start=std::chrono::high_resolution_clock::now();

    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in

    // Get relative target angle
    double yaw = tf2::getYaw(pose_.orientation);

    while (yaw>M_PI){
        yaw -=2*M_PI;
    }

    while (yaw<-M_PI){
        yaw +=2*M_PI;
    }

    double theta = yaw - target_angle_;

    double dyaw = target_angle_ - yaw;

    // Move at `speed` in target direction
    double dx = TARGET_SPEED * std::cos(theta);
    double dy = TARGET_SPEED * std::sin(theta);

    double dz;

    if(pose_.position.z>(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
        dz=-0.05;
    }
    if(pose_.position.z<(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
        dz=+0.05;
    }

    bool reached = goalReached();  

    if((reached)||(emergency_stop)){
        goalSet_=false;
        // Stop the quadcopter immediately
        sendCmd(0, 0, 0, 0);
        status_=pfms::PlatformStatus::IDLE;
        // sendLanding();
        ROS_INFO("Goal reached");
    }
    else{
        //Let's send command with these parameters
        sendCmd(dyaw, -dy, dz, dx);
        // RCLCPP_INFO_STREAM(get_logger(),"sending: " << dx << " " << -dy << " " << dz);
        // RCLCPP_INFO_STREAM(get_logger(),"sending: " << theta);
    }
        // RCLCPP_INFO_STREAM(get_logger(),"Time: " << time_travelled_);
    
    updateTimeInMotion();
    updateDistanceTravelled();

    return reached;
}



