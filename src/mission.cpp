#include "mission.h"
#include <iostream>

Mission::Mission() : Node("mission"),
                     mode_(mission::mode::BASIC),
                     previous_gndHeight(999),
                     tolerance_(0.5),
                     ct_(0),
                     ongoing_goal(false),
                     drone_chasing_(false),
                     orange_chasing_(false),
                     mission_status(false),
                     eval_done(false),
                     advanced_mode_(false)
{

    //! By default this code will save goals from this package to GOALS.txt where you run the code <br>
    //! rosrun project_setup goals_logger
    //!
    //! You can also supply a file of goals yourself <br>
    //! ros2 run a3_support goals_logger --ros-args -p filename:=$HOME/GOALS.TXT
    //! In above this saves the goals to GOALS.TXT in your home directory
    // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    // param_desc.description = "Filename to save goals";
    // node_handle_->declare_parameter("filename", "GOALS.txt", param_desc);
    // std::string filename = node_handle_->get_parameter("filename").as_string();
    // RCLCPP_INFO_STREAM(node_handle_->get_logger(),"file name with goals to be saved:" << filename);

    this->declare_parameter<bool>("_advanced", false);
    advanced_mode_ = this->get_parameter("_advanced").as_bool();
    if (advanced_mode_ == true)
    {
        mode_ = mission::mode::ADVANCED;
    }

    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/mission/goals", 100, std::bind(&Mission::goalsCallback, this, std::placeholders::_1));
    sub_drone_odo_ = this->create_subscription<nav_msgs::msg::Odometry>("drone/gt_odom", 100, std::bind(&Mission::odomCallback_drone, this, std::placeholders::_1));
    sub_orange_odo_ = this->create_subscription<nav_msgs::msg::Odometry>("/orange/odom", 100, std::bind(&Mission::odomCallback_orange, this, std::placeholders::_1));
    sub_sonar_ = this->create_subscription<sensor_msgs::msg::Range>("/drone/sonar", 100, std::bind(&Mission::sonarCallback_, this, std::placeholders::_1));

    pub_visual_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1000);
    pub_drone_goal_ = this->create_publisher<geometry_msgs::msg::Point>("/drone/goal", 10);
    pub_orange_goal_ = this->create_publisher<geometry_msgs::msg::Point>("/orange/goal", 10);

    pub_waypoint = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission/path", 10);
    pub_danger = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission/danger", 10);

    srv1_mission_ = this->create_service<std_srvs::srv::SetBool>(
        "/mission/control", std::bind(&Mission::control, this, std::placeholders::_1, std::placeholders::_2));

    // We create as an example here a function that is called via a timer at a set interval
    // Create a timer that calls the timerCallback function every 500ms
    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Mission::progress, this));

    // A thread that will run the function threadFunction
    // The thread could start before any data is recieved via callbacks as we have not called spin yet
    thread_ = new std::thread(&Mission::run, this);
}

Mission::~Mission()
{
    if (thread_->joinable())
    {
        thread_->join();
    }
    delete thread_;
}

void Mission::sonarCallback_(const std::shared_ptr<sensor_msgs::msg::Range> msg)
{
    std::unique_lock<std::mutex> lck(mtxSonar);
    sonarData = *msg;
    lck.unlock();
}


void Mission::visualizeWaypoints(){
        std::deque<geometry_msgs::msg::Point> intermediate_points;
        drone_goals_.push_front(drone_pose_.pose.pose.position);
        for (long unsigned int i = 0; i < drone_goals_.size() - 1; ++i)
        {
            auto waypoints = generateIntermediateWaypoints(drone_goals_.at(i), drone_goals_.at(i + 1), 1.0);
            intermediate_points.insert(intermediate_points.end(), waypoints.begin(), waypoints.end());
        }
        drone_goals_.pop_front();

        for (auto goal : intermediate_points)
        {
            visualization_msgs::msg::Marker marker = produceMarker(goal, mission::roadState::WAYPOINT);
            markerArray_.markers.push_back(marker);
            pub_visual_->publish(markerArray_);
        }
}


void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{

    pt_vec = msg->poses;
    geometry_msgs::msg::Point pt;


    switch (mode_)
    {
    case (mission::mode::BASIC):

        for (auto currentpt : pt_vec)
        {
            pt.x = currentpt.position.x;
            pt.y = currentpt.position.y;
            pt.z = currentpt.position.z;

            visualization_msgs::msg::Marker marker = produceMarker(pt, mission::roadState::DRONE_PATH);
            markerArray_.markers.push_back(marker);
            pub_visual_->publish(markerArray_);

            drone_goals_.push_back(pt);
            RCLCPP_INFO_STREAM(this->get_logger(), "Have: " << drone_goals_.size() << " goals");
        }
        ////////////// Generate intermediate waypoints
        visualizeWaypoints();
        ////////////
        break;

    case (mission::mode::ADVANCED):

        for (auto currentpt : pt_vec)
        {
            pt.x = currentpt.position.x;
            pt.y = currentpt.position.y;
            pt.z = currentpt.position.z;

            visualization_msgs::msg::Marker marker = produceMarker(pt, mission::roadState::DRONE_PATH);
            markerArray_.markers.push_back(marker);
            pub_visual_->publish(markerArray_);

            drone_goals_.push_back(pt);
        }


        // Set the start as the current pose_
        geometry_msgs::msg::Point start_point;
        start_point.x = drone_pose_.pose.pose.position.x;
        start_point.y = drone_pose_.pose.pose.position.y;
        start_point.z = drone_pose_.pose.pose.position.z;

        // Create a vector of pointers to goals including the start and end points
        // I don't know if this is right, but working with pointers should be faster than supplying it the
        // values, therefore pointers, idk if this is right. Tada~~
        std::vector<geometry_msgs::msg::Point *> goal_pointers;
        goal_pointers.push_back(&start_point);
        for (auto &goal : drone_goals_)
        {
            goal_pointers.push_back(&goal);
        }

        // Perform TSP search
        int start = 0;
        int end = goal_pointers.size() - 1;

        TSP_search tsp(goal_pointers, start, end);
        std::vector<int> optimal_order = tsp.brute_force_tsp();

        // Create the ordered goals vector excluding the start and end point for drone_goals_
        std::deque<geometry_msgs::msg::Point> ordered_goals;

        for (int idx : optimal_order)
        {
            if (idx != start)
            {
                ordered_goals.push_back(*goal_pointers.at(idx));
            }
        }

        drone_goals_ = ordered_goals;

        visualizeWaypoints();

        break;
    }
}

void Mission::odomCallback_drone(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::unique_lock<std::mutex> lck(mtxOdo_drone_);
    drone_pose_ = *msg;
    lck.unlock();
}

void Mission::odomCallback_orange(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::unique_lock<std::mutex> lck(mtxOdo_orange_);
    orange_pose_ = *msg;
    lck.unlock();
}

// Takes 10 possible readings than averages them
float Mission::calHeight()
{
    sonarCount = 0;
    sumSonar = 0;
    for (unsigned int i = 0; i < 10; i++)
    {
        std::unique_lock<std::mutex> lck(mtxSonarData);
        sonarRange_ = sonarData.range;
        max_Range_ = sonarData.max_range;
        min_Range_ = sonarData.min_range;
        lck.unlock();

        if ((sonarRange_ < max_Range_) && (sonarRange_ > min_Range_) &&
            !std::isnan(sonarRange_) && std::isfinite(sonarRange_))
        {
            sumSonar += sonarRange_;
            sonarCount += 1;
        }
    }
    avgSonar = sumSonar / sonarCount;
    return drone_pose_.pose.pose.position.z - avgSonar;
}

void Mission::evaluateGradient(float initial_height, float final_height, float distance, geometry_msgs::msg::Point goalInput)
{

    // gradient = fabs(final_height - initial_height) / distance;
    dataEval.evaluateGradient(initial_height, final_height, distance);
    goalInput.z = final_height;
    visualization_msgs::msg::Marker marker;

    if (gradient < 0.03)
    {
        std::unique_lock<std::mutex> lock(mtxOrangeGoals_);
        switch (mode_)
        {
        case (mission::mode::BASIC):
            orange_goals_.push_back(drone_goals_.front());
            break;
        case (mission::mode::ADVANCED):
            orange_unordered_goals_.push_back(drone_goals_.front());
            break;
        }
        lock.unlock();
        marker = produceMarker(goalInput, mission::roadState::ROAD);
    }

    else if (gradient >= 0.03)
    {
        marker = produceMarker(goalInput, mission::roadState::DANGER);
        orange_danger_point.push_back(drone_goals_.front());
        mission_status = false;
    }

    visualization_msgs::msg::MarkerArray marker_Array_;
    marker_Array_.markers.push_back(marker);
    pub_visual_->publish(marker_Array_);

    eval_done = true;
    cond_var.notify_one();
}

void Mission::cmd_drone_goal(geometry_msgs::msg::Point goal)
{
    pub_drone_goal_->publish(goal);
}

void Mission::cmd_orange_goal(geometry_msgs::msg::Point goal)
{
    pub_orange_goal_->publish(goal);
}

void Mission::progress()
{

    if (drone_chasing_)
    {
        nav_msgs::msg::Odometry drone_odo;
        geometry_msgs::msg::Point goal_drone;
        std::unique_lock<std::mutex> lck1(mtxOdo_drone_);
        drone_odo = drone_pose_;
        lck1.unlock();
        std::unique_lock<std::mutex> lck2(mtxDroneGoals_);
        goal_drone = drone_goals_.front();
        lck2.unlock();
        double dist_drone = distance(drone_odo, goal_drone);
        progressDrone = (unsigned int)(100.0 * ((init_dist_to_goal_drone_ - dist_drone) / init_dist_to_goal_drone_));

        nav_msgs::msg::Odometry orange_odo;
        geometry_msgs::msg::Point goal_orange;
        std::unique_lock<std::mutex> lck3(mtxOdo_orange_);
        orange_odo = orange_pose_;
        lck3.unlock();
        std::unique_lock<std::mutex> lck4(mtxDroneGoals_);
        goal_orange = orange_goals_.front();
        lck4.unlock();
        double dist_orange = distance(orange_odo, goal_orange);
        progressOrange = (unsigned int)(100.0 * ((init_dist_to_goal_orange_ - dist_orange) / init_dist_to_goal_orange_));

    }
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt)
{
    return std::pow((std::pow(odo.pose.pose.position.x - pt.x, 2) + std::pow(odo.pose.pose.position.y - pt.y, 2)), 0.5);
}

void Mission::run()
{

    // This function runs in a separate thread of execution, it is started in the constructor
    // Make sure that this is thread safe
    // This function needs to terminate when the node is destroyed and we can do so by checking if the node is still alive
    while (rclcpp::ok())
    {

        nav_msgs::msg::Odometry drone_odo;
        geometry_msgs::msg::Point drone_goal;
        nav_msgs::msg::Odometry orange_odo;
        geometry_msgs::msg::Point orange_goal;

        //=====================  Drone  =======================

        if (drone_goals_.size() > 0)
        {

            std::unique_lock<std::mutex> lck1(mtxOdo_drone_);
            drone_odo = drone_pose_;
            lck1.unlock();

            std::unique_lock<std::mutex> lck2(mtxDroneGoals_);
            drone_goal = drone_goals_.front();
            lck2.unlock();

            if (!ongoing_goal)
            {
                dis_drone_goals_ = distance(drone_odo, drone_goal);

                ongoing_goal = true;
                if (newGoals)
                {
                    current_gndHeight = orange_pose_.pose.pose.position.z;
                }
            }
            if (mode_ == mission::mode::ADVANCED)
            {

                if (orange_unordered_goals_.size() > 0)
                {
                    geometry_msgs::msg::Point start_point;
                    start_point.x = orange_pose_.pose.pose.position.x;
                    start_point.y = orange_pose_.pose.pose.position.y;
                    start_point.z = orange_pose_.pose.pose.position.z;

                    std::vector<geometry_msgs::msg::Point *> goal_pointers;
                    goal_pointers.push_back(&start_point);
                    for (auto &goal : orange_unordered_goals_)
                    {
                        goal_pointers.push_back(&goal);
                    }

                    // Perform TSP search
                    int start = 0;
                    int end = goal_pointers.size() - 1;

                    TSP_search tsp(goal_pointers, start, end);
                    std::vector<int> optimal_order = tsp.brute_force_tsp();

                    std::deque<geometry_msgs::msg::Point> ordered_goals;

                    for (int idx : optimal_order)
                    {
                        if (idx != start)
                        {
                            ordered_goals.push_back(*goal_pointers.at(idx));
                        }
                    }

                    orange_goals_ = ordered_goals;
                }
            }
        }
        //=====================  Orange  =======================
        if (orange_goals_.size() > 0)
        {

            std::unique_lock<std::mutex> lck3(mtxOdo_orange_);
            orange_odo = orange_pose_;
            lck3.unlock();

            std::unique_lock<std::mutex> lck4(mtxOrangeGoals_);
            orange_goal = orange_goals_.front();
            lck4.unlock();
        }

        if (mission_status)
        {

            // std
            if ((drone_chasing_))
            {
                double drone_dist = distance(drone_odo, drone_goal);

                if (drone_dist < tolerance_)
                {
                    previous_gndHeight = current_gndHeight;
                    current_gndHeight = calHeight();
                    {
                        std::unique_lock<std::mutex> lock(cond_mtx);
                        eval_done = false;
                        evaluateGradient(previous_gndHeight, current_gndHeight, dis_drone_goals_, drone_goal);
                        cond_var.wait(lock, [this]
                                      { return eval_done; });
                    }

                    ongoing_goal = false;
                    std::unique_lock<std::mutex> lock5(mtxDroneGoals);
                    drone_goals_.pop_front(); // Removoing element at front
                    lock5.unlock();
                    RCLCPP_INFO(this->get_logger(), "Drone Reached And Going For New Goal");
                    if (drone_goals_.size() > 0)
                    {
                        drone_goal = drone_goals_.front();
                        init_dist_to_goal_drone_ = distance(drone_odo, drone_goal);
                        pub_drone_goal_->publish(drone_goal);
                    }
                }

                if (drone_goals_.size() == 0)
                {
                    drone_chasing_ = false;
                }
            }
            else
            {
                if (drone_goals_.size() > 0)
                {
                    drone_chasing_ = true;
                    drone_goal = drone_goals_.front();
                    init_dist_to_goal_drone_ = distance(drone_odo, drone_goal);
                    pub_drone_goal_->publish(drone_goal);
                    RCLCPP_INFO(this->get_logger(), "Drone Sending First Goal");
                }
            }

            if ((orange_chasing_))
            {

                double orange_dist = distance(orange_odo, orange_goal);

                if (orange_dist < tolerance_)
                {
                    orange_goals_.pop_front(); // Removoing element at front
                    RCLCPP_INFO(this->get_logger(), "Car Fetching Next Goal!");

                    if (orange_goals_.size() > 0)
                    {
                        orange_goal = orange_goals_.front();
                        init_dist_to_goal_orange_ = distance(drone_odo, drone_goal);
                        pub_orange_goal_->publish(orange_goal);
                    }

                    if (orange_goals_.size() == 0)
                    {
                        orange_chasing_ = false;
                    }
                }
            }
            else
            {
                if (orange_goals_.size() > 0)
                {
                    orange_chasing_ = true;
                    orange_goal = orange_goals_.front();
                    init_dist_to_goal_orange_ = distance(drone_odo, drone_goal);
                    pub_orange_goal_->publish(orange_goal);
                    RCLCPP_INFO(this->get_logger(), "Car Initiating! Vroom Vroom! 🚗🚗");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // we sleep for 10 miliseconds
        // break;
    }
}

void Mission::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{

    // chasing_
    if (req->data)
    {
        mission_status = true;
        newGoals = true;
    }
    else if (!req->data)
    {
        mission_status = false;
        //     goalSet_=false;
        drone_goals_.clear();
        orange_goals_.clear();
        orange_unordered_goals_.clear();
    }

    if ((drone_goals_.size() == 0) && (orange_goals_.size() == 0))
    {
        res->message = "No Goals Set";
    }

    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Progress:" << progressDrone);
    }

    res->success = mission_status;
}

std::vector<geometry_msgs::msg::Point> Mission::generateIntermediateWaypoints(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, double interval)
{
    std::vector<geometry_msgs::msg::Point> waypoints;
    double distance = std::hypot(goal.x - start.x, goal.y - start.y); // Standard distance formula
    int num_waypoints = static_cast<int>(distance / interval);        // Calculates how many goals there are

    for (int i = 1; i <= num_waypoints; ++i)
    {
        geometry_msgs::msg::Point waypoint;
        waypoint.x = start.x + i * (goal.x - start.x) / num_waypoints;
        waypoint.y = start.y + i * (goal.y - start.y) / num_waypoints;
        waypoint.z = start.z + i * (goal.z - start.z) / num_waypoints;
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

visualization_msgs::msg::Marker Mission::produceMarker(geometry_msgs::msg::Point pt, mission::roadState state)
{

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.lifetime = rclcpp::Duration(1000, 0); // zero is forever

    std_msgs::msg::ColorRGBA color;

    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = pt.z;

    // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.id = ct_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

    switch (state)
    {

    case (mission::roadState::DRONE_PATH):
        // The marker type
        marker.ns = "Drone_Path"; // This is namespace, markers can be in different namespace
        marker.type = visualization_msgs::msg::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.5;

        color.a = 0.8;
        color.r = 0.0;
        color.g = 0.8;
        color.b = 0.0;

        break;

    case (mission::roadState::ROAD):
        // The marker type
        marker.ns = "Road"; // This is namespace, markers can be in diofferent namespace
        marker.type = visualization_msgs::msg::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;

        // Let's send a marker with color (green for reachable, red for now)
        color.a = 0.8; // a is alpha - transparency 0.5 is 50%;
        color.r = 0;
        color.g = 1;
        color.b = 250.0 / 255.0;

        break;

    case (mission::roadState::DANGER):
        // The marker type
        marker.ns = "Danger"; // This is namespace, markers can be in diofferent namespace
        marker.type = visualization_msgs::msg::Marker::CUBE;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Let's send a marker with color (green for reachable, red for now)
        color.a = 0.8; // a is alpha - transparency 0.5 is 50%;
        color.r = 1;
        color.g = 0;
        color.b = 0;

        break;

    case (mission::roadState::WAYPOINT):
        // The marker type
        marker.ns = "Waypoint"; // This is namespace, markers can be in diofferent namespace
        marker.type = visualization_msgs::msg::Marker::CYLINDER;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the marker -- 1m side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Let's send a ma
        color.a = 0.5; 
        color.g = 0;
        color.b = 0.5;

        break;
    }

    marker.color = color;
    return marker;
}