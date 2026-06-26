#include "ackerman.h"
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

Ackerman::Ackerman() : Node("ackerman_control"),
                       TARGET_SPEED(0.4),
                       mass(1500),
                       brake_force(3000)
{
    tolerance_ = 0.9; // We set tolerance to be default of 0.5

    /*I don't know what's happening here. This top section I put in so I can properly run
    "/clock" properly or else I get the following warning:

    "New publisher discovered on topic '/clock', offering incompatible QoS.
    No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY"

    */
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 1000, std::bind(&Ackerman::odoCallback, this, _1));
    sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/orange/goal", 1000, std::bind(&Ackerman::setGoal, this, _1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/orange/twist", 1000, std::bind(&Ackerman::velocityCallback, this, _1));

    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 1000, std::bind(&Ackerman::laserCallback, this, _1));

    // sub_clock_ = this->create_subscription<rosgraph_msgs::msg::Clock> (
    //     "/clock", qos_profile, std::bind(&Ackerman::clockCallback,this, _1));

    pubEmergencyStop_ = this->create_publisher<std_msgs::msg::Bool>("/orange/emergency_stop", 3);
    pubBrake_ = this->create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd", 3);
    pubSteering_ = this->create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 3);
    pubThrottle_ = this->create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 3);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Ackerman::reachGoal, this));
};

Ackerman::~Ackerman()
{
    timer_->cancel(); // Cancel the timer
    RCLCPP_INFO(get_logger(), "Ackerman node shutting down. Goodnight Folks!");
}

bool Ackerman::checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double &distance, double &time,
                                        geometry_msgs::msg::Pose &estimatedGoalPose)
{

    currentOdo_ = poseToOdo(origin);
    estimatedGoalOdo_ = poseToOdo(estimatedGoalPose);
    audi_reachable = audi.checkOriginToDestination(currentOdo_, {goal.x, goal.y, goal.z}, distance, time, estimatedGoalOdo_);
    estimatedGoalPose = odoToPose(estimatedGoalOdo_);

    return audi_reachable;
}

bool Ackerman::calcNewGoal(void)
{

    getOdometry(); // This will update internal copy of odometry, as well as return value if needed.
    currentOdo_ = poseToOdo(pose_);

    if (!checkOriginToDestination(pose_, goal_.location, goal_.distance, goal_.time, est_final_pose))
        return false;

    audi.computeSteering(currentOdo_, pfmsGoal, steering_, goal_.distance);
    updateBrakeInfo();

    return true;
}

void Ackerman::sendAckCmd(double brake, double steering, double throttle)
{
    brake_msg.set__data(brake);
    steering_msg.set__data(steering);
    throttle_msg.set__data(throttle);

    pubBrake_->publish(brake_msg);
    pubSteering_->publish(steering_msg);
    pubThrottle_->publish(throttle_msg);
}

void Ackerman::odoCallback(const nav_msgs::msg::Odometry &msg)
{
    std::unique_lock<std::mutex> lock(poseMtx_);
    pose_.set__orientation(msg.pose.pose.orientation);
    pose_.set__position(msg.pose.pose.position);
}

bool Ackerman::reachGoal(void)
{

    if (!goalSet_)
    {
        return false;
    };

    getOdometry();
    currentOdo_ = poseToOdo(pose_);

    if (!checkOriginToDestination(pose_, goal_.location, goal_.distance, goal_.time, est_final_pose))
    {
        RCLCPP_INFO_STREAM(get_logger(), "Audi Goal Not Reachable ");
        goalSet_ = false;
    }

    if ((goalSet_) && (status_ == pfms::PlatformStatus::IDLE))
    {
        status_ = pfms::PlatformStatus::RUNNING;
        ackermanState = ackerman::State::Accelerate;
    }

    time_start = std::chrono::high_resolution_clock::now();

    if (goalSet_)
    {

        for (auto laserRange : laserData.ranges)
        {

            if ((laserRange < laserData.range_max) && (laserRange > laserData.range_min) &&
                !std::isnan(laserRange) && std::isfinite(laserRange) && (laserRange < 1.0))
            {
                ackermanState = ackerman::State::Decelerate;
                emergency_stop = true;
                
                std_msgs::msg::Bool emergency_stop_msg;
                emergency_stop_msg.set__data(true);
                pubEmergencyStop_->publish(emergency_stop_msg);
                RCLCPP_INFO_STREAM(get_logger(), "🚨 Alert! Object detected at " << laserRange << " metres");
                break;
            }
        }
        updateDistanceTravelled();
        calcNewGoal();

        switch (ackermanState)
        {
        case (ackerman::State::Accelerate):
            if (throttle_ < 0.8)
            {
                throttle_ += 0.02;
                brake_ = 0;
            }

            if ((velocity > 2.0))
            {
                throttle_ = 0.0;
            }

            if ((velocity > 2.5) && (goal_.distance < 3.0))
            {
                throttle_ = 0.0;
                brake_ = 1000;
            }

            if ((updateBrakeInfo()) || (goal_.distance <= tolerance_))
            {
                throttle_ = 0;
                ackermanState = ackerman::State::Decelerate;
            }

            break;

        case (ackerman::State::Decelerate):
            if (brake_ < 8000)
            {
                brake_ = brake_force;
            }

            if ((goal_.distance <= tolerance_) || (emergency_stop))
            {
                brake_ = 80000;
                throttle_ = 0;
                steering_ = 0;
            }

            if (velocity <= 0.1)
            {
                ackermanState = ackerman::State::Stopped;
            }

            [[fallthrough]]; // My first time using this, this is so cool omg.

        case (ackerman::State::Stopped):
            status_ = pfms::PlatformStatus::IDLE;
            goalSet_ = false;
            break;
        }

        sendAckCmd(brake_, steering_, throttle_);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    updateTimeInMotion();
    return true;
}

pfms::nav_msgs::Odometry Ackerman::poseToOdo(geometry_msgs::msg::Pose poseConv_)
{
    odo_.position.x = poseConv_.position.x;
    odo_.position.y = poseConv_.position.y;
    odo_.position.z = poseConv_.position.z;
    odo_.yaw = tf2::getYaw(poseConv_.orientation);

    return odo_;
}

void Ackerman::velocityCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    std::unique_lock<std::mutex> lock(velMtx_);
    velocity = msg.twist.linear.x;
}

geometry_msgs::msg::Pose Ackerman::odoToPose(pfms::nav_msgs::Odometry odoConv_)
{
    // pfms::nav_msgs::Odometry odo_;
    tf2::Quaternion q;
    q.setRPY(0, 0, odoConv_.yaw);
    geometry_msgs::msg::Pose poseConv_;

    poseConv_.position.x = odoConv_.position.x;
    poseConv_.position.y = odoConv_.position.y;
    poseConv_.position.z = odoConv_.position.z;
    poseConv_.orientation.w = q.getW();
    poseConv_.orientation.x = q.getX();
    poseConv_.orientation.y = q.getY();
    poseConv_.orientation.z = q.getZ();

    return poseConv_;
}

bool Ackerman::updateBrakeInfo()
{
    work = 0.5 * mass * velocity;
    return (goal_.distance <= (work / brake_force));
}
