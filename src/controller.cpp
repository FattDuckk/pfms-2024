#include "controller.h"
#include <cmath>



/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    
    goalSet_(false),    
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0),
    emergency_stop(false),
    status_(pfms::PlatformStatus::IDLE)
{

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
    previous_pose_empty=true;
};


bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

void Controller::updateDistanceTravelled(){
    if (previous_pose_empty){
        previous_pose_=pose_;
        previous_pose_empty = false;
    }
    distance_travelled_+=pow(pow((pose_.position.x - previous_pose_.position.x),2)+pow((pose_.position.y - previous_pose_.position.y),2),0.5);
    previous_pose_=pose_;
}

void Controller::updateTimeInMotion(){
    time_stop=std::chrono::high_resolution_clock::now();
    auto duration_ = std::chrono::duration<double>(time_stop - time_start);
    time_travelled_+=duration_.count()*10;
    
}


bool Controller::goalReached() {
    double dx = goal_.location.x - pose_.position.x;
    double dy = goal_.location.y - pose_.position.y;
    double dz = goal_.location.z - pose_.position.z;

    return (pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5) < tolerance_);
}

void Controller::odoCallback(const nav_msgs::msg::Odometry& msg){
    std::unique_lock<std::mutex> lock(poseMtx_);
    pose_ = msg.pose.pose;
}

void Controller::laserCallback(const sensor_msgs::msg::LaserScan& msg){
    std::unique_lock<std::mutex> lck(laserMtx_);
    laserData = msg;
    lck.unlock();  
}

void Controller::setGoal(const geometry_msgs::msg::Point& msg){    
  goal_.location = msg;
  pfmsGoal = {goal_.location.x, goal_.location.y, 0};
  goalSet_=true;
  emergency_stop = false;
}

geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::unique_lock<std::mutex> lock(poseMtx_);
    return pose_;
}

std::string Controller::getInfoString()
{
    std::stringstream ss;
    switch(status_)
    {
        case pfms::PlatformStatus::IDLE   : ss << "IDLE ";    break;
        case pfms::PlatformStatus::RUNNING : ss << "RUNNING ";  break;
        case pfms::PlatformStatus::TAKEOFF : ss << "TAKEOFF ";  break;
        case pfms::PlatformStatus::LANDING : ss << "LANDING ";  break;
    }
    return ss.str(); // This command convertes trsingstream to string
}