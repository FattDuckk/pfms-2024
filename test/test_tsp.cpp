// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// You will need to add headers for any other messages that you use
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "../src/tsp_search.h"

// We state the test suite name and the test name
TEST(testTSP, TSP)
{
    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package

    /* Goals used
    ros2 topic pub - 1 / mission / goals geometry_msgs / msg / PoseArray "{
                         poses : [
                             {position : {x : 1.0, y : 2.0, z : 2.0}, orientation : {x : 0.0, y : 0.0, z : 0.0, w : 1.0}},
                             {position : {x : 2.0, y : -3.0, z : 2.0}, orientation : {x : 0.0, y : 0.0, z : 0.0, w : 1.0}},
                             {position : {x : 3.0, y : 4.0, z : 2.0}, orientation : {x : 0.0, y : 0.0, z : 0.0, w : 1.0}},
                             {position : {x : 4.0, y : -5.0, z : 2.0}, orientation : {x : 0.0, y : 0.0, z : 0.0, w : 1.0}},
                             {position : {x : 5.0, y : 6.0, z : 2.0}, orientation : {x : 0.0, y : 0.0, z : 0.0, w : 1.0}}
                         ]
    }"
    */

    // Drone position at      x: -3,  y: 0,  z: 2

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_submission"); // If you chnage package name you need to change this
    std::string bag_filename = package_share_directory + "/data/rosbags/utest3Data";                     // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    rclcpp::Serialization<nav_msgs::msg::Odometry> drone_odo_serialization;
    nav_msgs::msg::Odometry::SharedPtr drone_odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    rclcpp::Serialization<geometry_msgs::msg::PoseArray> goals_serialization;
    geometry_msgs::msg::PoseArray::SharedPtr goals_msg = std::make_shared<geometry_msgs::msg::PoseArray>();

    reader.open(bag_filename);

    bool goals_found = false;
    bool d_odo_found = false;

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if ((msg->topic_name == "/mission/goals") && (!goals_found))
        {
            rclcpp::SerializedMessage serialized_goals_msg(*msg->serialized_data);
            goals_serialization.deserialize_message(&serialized_goals_msg, goals_msg.get());
            goals_found = true;
        }

        if ((msg->topic_name == "/drone/gt_odom") && (!d_odo_found))
        {
            rclcpp::SerializedMessage serialized_drone_odo_msg(*msg->serialized_data);
            drone_odo_serialization.deserialize_message(&serialized_drone_odo_msg, drone_odo_msg.get());
            d_odo_found = true;
        }

        if ((d_odo_found) && (goals_found))
        {
            break;
        }
    }

    geometry_msgs::msg::Point drone_odo_ = drone_odo_msg->pose.pose.position;
    geometry_msgs::msg::Point pt;

    geometry_msgs::msg::PoseArray goals = *goals_msg;
    std::deque<geometry_msgs::msg::Point> drone_goals_;
    for (auto pose : goals_msg->poses)
    {
        drone_goals_.push_back(pose.position);
    }

    geometry_msgs::msg::Point start_point;
    start_point.x = drone_odo_.x;
    start_point.y = drone_odo_.y;
    start_point.z = drone_odo_.z;

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
    std::vector<int> optimal_order_cleaned;

    // Create the ordered goals vector excluding the start and end point for drone_goals_
    std::deque<geometry_msgs::msg::Point> ordered_goals;

    for (int idx : optimal_order)
    {
        if (idx != start)
        {
            ordered_goals.push_back(*goal_pointers.at(idx));
            idx -= 1;
            optimal_order_cleaned.push_back(idx);
        }
    }

    std::vector<int> expected_order = {1, 3, 0, 2, 4};

    EXPECT_TRUE(optimal_order_cleaned.size() == expected_order.size());
    // bool equal = (optimal_order = expected_order);

    for (long unsigned int i = 0; i < optimal_order_cleaned.size(); i++)
    {
        std::cout << "Output Order: " << optimal_order_cleaned.at(i) << " Expected Order: " << expected_order.at(i) << std::endl;
    }

    EXPECT_TRUE(optimal_order_cleaned == expected_order);
}
