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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "../src/dataEvaluation.h"

// We state the test suite name and the test name
TEST(testBasicGoal, Reachable)
{
    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_submission"); // If you chnage package name you need to change this
    std::string bag_filename = package_share_directory + "/data/rosbags/utest2Data";                     // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    rclcpp::Serialization<nav_msgs::msg::Odometry> drone_odo_serialization;
    nav_msgs::msg::Odometry::SharedPtr drone_odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    rclcpp::Serialization<nav_msgs::msg::Odometry> orange_odo_serialization;
    nav_msgs::msg::Odometry::SharedPtr orange_odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    rclcpp::Serialization<sensor_msgs::msg::Range> sonar_serialization;
    sensor_msgs::msg::Range::SharedPtr sonar_msg = std::make_shared<sensor_msgs::msg::Range>();

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> laserscan_serialization;
    sensor_msgs::msg::LaserScan::SharedPtr laserscan__msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we have the one message
    // If you need multiple messages you need to adjust this loop to break when your have both messages


    bool sonar_found = false;
    bool d_odo_found = false;
    bool o_odo_found = false;
    bool laser_found = false;

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if ((msg->topic_name == "/drone/sonar") && (!sonar_found))
        {
            rclcpp::SerializedMessage serialized_sonar_msg(*msg->serialized_data);
            sonar_serialization.deserialize_message(&serialized_sonar_msg, sonar_msg.get());
            sonar_found = true;
        }

        if ((msg->topic_name == "/drone/laserscan") && (!laser_found))
        {
            rclcpp::SerializedMessage serialized_laser_msg(*msg->serialized_data);
            laserscan_serialization.deserialize_message(&serialized_laser_msg, laserscan__msg.get());
            laser_found = true;

        }

        if ((msg->topic_name == "/drone/gt_odom") && (!d_odo_found))
        {
            rclcpp::SerializedMessage serialized_drone_odo_msg(*msg->serialized_data);
            drone_odo_serialization.deserialize_message(&serialized_drone_odo_msg, drone_odo_msg.get());
            d_odo_found = true;

        }

        if ((msg->topic_name == "/orange/odom") && (!o_odo_found))
        {
            rclcpp::SerializedMessage serialized_orange_odo_msg(*msg->serialized_data);
            orange_odo_serialization.deserialize_message(&serialized_orange_odo_msg, orange_odo_msg.get());
            o_odo_found = true;

        }


        if (((sonar_found) && (d_odo_found)) && ((o_odo_found) && (laser_found)))
        {
            break;
        }

    }


    float sonar_height = sonar_msg->range;
    sensor_msgs::msg::LaserScan laser = *laserscan__msg;
    geometry_msgs::msg::Point drone_odo_ = drone_odo_msg->pose.pose.position;

    geometry_msgs::msg::Point orange_odo_ = orange_odo_msg->pose.pose.position;

    float height_1 = orange_odo_.z;
    float height_2 = sonar_height - drone_odo_.z;
    float distanceDiff_1;

    bool emergency_stop = false;
    bool small_gradient;

    distanceDiff_1 = sqrt(pow(drone_odo_.x - orange_odo_.x, 2) + pow(drone_odo_.y - orange_odo_.y, 2));

    DataEvaluator dataEvaluator;
    
    double gradient = dataEvaluator.evaluateGradient(height_1, height_2, distanceDiff_1);

    std::cout << "Calculated Gradient: "<< gradient << std::endl;

    //Modified function from mission.cpp
    //Emergency stop if the detect object is less than 0.1m
    for (auto laserRange : laser.ranges)
    {

        if ((laserRange < laser.range_max) && (laserRange > 0.15) &&
            !std::isnan(laserRange) && std::isfinite(laserRange) && (laserRange < 0.5))
        {
            emergency_stop = true;               
            break;
        }
    }

    small_gradient = gradient < 0.03;

    EXPECT_TRUE(small_gradient&&(!emergency_stop));

}


