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
#include "../src/dataEvaluation.h"

// We state the test suite name and the test name
TEST(testBasicGoal, Reachable)
{
    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_submission"); // If you chnage package name you need to change this
    std::string bag_filename = package_share_directory + "/data/rosbags/droneData1";                     // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<sensor_msgs::msg::Range> sonar_serialization;
    // We also need a message object (shared pointer in this case)
    sensor_msgs::msg::Range::SharedPtr sonar_msg = std::make_shared<sensor_msgs::msg::Range>();

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<nav_msgs::msg::Odometry> odo_serialization;
    // We also need a message object (shared pointer in this case)
    nav_msgs::msg::Odometry::SharedPtr odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we have the one message
    // If you need multiple messages you need to adjust this loop to break when your have both messages

    rosbag2_cpp::Reader reader2; // Create a reader object
    std::string bag_filename2 = package_share_directory + "/data/rosbags/droneData2";                     // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rclcpp::Serialization<sensor_msgs::msg::Range> sonar2_serialization;
    sensor_msgs::msg::Range::SharedPtr sonar2_msg = std::make_shared<sensor_msgs::msg::Range>();

    rclcpp::Serialization<nav_msgs::msg::Odometry> odo2_serialization;
    nav_msgs::msg::Odometry::SharedPtr odo2_msg = std::make_shared<nav_msgs::msg::Odometry>();


    reader2.open(bag_filename2);

    bool sonar_found = false;
    bool odo_found = false;

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if ((msg->topic_name == "/drone/sonar") && (!sonar_found))
        {
            rclcpp::SerializedMessage serialized_sonar_msg(*msg->serialized_data);
            sonar_serialization.deserialize_message(&serialized_sonar_msg, sonar_msg.get());
            sonar_found = true;
        }

        if ((msg->topic_name == "/drone/gt_odom") && (!odo_found))
        {
            rclcpp::SerializedMessage serialized_odo_msg(*msg->serialized_data);
            odo_serialization.deserialize_message(&serialized_odo_msg, odo_msg.get());
            odo_found = true;

        }
        
        if (sonar_found && odo_found)
        {
            break;
        }
    }

    sonar_found = false;
    odo_found = false;

    while (reader2.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader2.read_next();

        if ((msg->topic_name == "/drone/sonar") && (!sonar_found))
        {
            rclcpp::SerializedMessage sonar2_serialization(*msg->serialized_data);
            sonar_serialization.deserialize_message(&sonar2_serialization, sonar2_msg.get());
            sonar_found = true;
        }

        if ((msg->topic_name == "/drone/gt_odom") && (!odo_found))
        {
            rclcpp::SerializedMessage odo2_serialization(*msg->serialized_data);
            odo_serialization.deserialize_message(&odo2_serialization, odo2_msg.get());
            odo_found = true;

            if (sonar_found && odo_found)
            {
                break;
            }
        }
    }

    float sonar_height = sonar_msg->range;
    geometry_msgs::msg::Point odo_ = odo_msg->pose.pose.position;

    float sonar_height_2 = sonar2_msg->range;
    geometry_msgs::msg::Point odo_2 = odo2_msg->pose.pose.position;

    float height_1 = sonar_height - odo_.z;
    float height_2 = sonar_height_2 - odo_2.z;
    float distanceDiff_1;

    distanceDiff_1 = sqrt(pow(odo_2.x - odo_.x, 2) + pow(odo_2.y - odo_.y, 2));

    DataEvaluator dataEvaluator;
    
    double gradient = dataEvaluator.evaluateGradient(height_1, height_2, distanceDiff_1);

    std::cout << "Calculated Gradient: "<< gradient << std::endl;

    EXPECT_TRUE(gradient < 0.03);

}

TEST(testBasicGoal, Not_Reachable)
{


    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_submission"); // If you chnage package name you need to change this
    std::string bag_filename = package_share_directory + "/data/rosbags/droneData1";                     // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    rclcpp::Serialization<sensor_msgs::msg::Range> sonar_serialization;
    sensor_msgs::msg::Range::SharedPtr sonar_msg = std::make_shared<sensor_msgs::msg::Range>();

    rclcpp::Serialization<nav_msgs::msg::Odometry> odo_serialization;
    nav_msgs::msg::Odometry::SharedPtr odo_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Open the bag file
    reader.open(bag_filename);

    bool sonar_found = false;
    bool odo_found = false;

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if ((msg->topic_name == "/drone/sonar") && (!sonar_found))
        {
            rclcpp::SerializedMessage serialized_sonar_msg(*msg->serialized_data);
            sonar_serialization.deserialize_message(&serialized_sonar_msg, sonar_msg.get());
            sonar_found = true;
        }

        if ((msg->topic_name == "/drone/gt_odom") && (!odo_found))
        {
            rclcpp::SerializedMessage serialized_odo_msg(*msg->serialized_data);
            odo_serialization.deserialize_message(&serialized_odo_msg, odo_msg.get());
            odo_found = true;

            if (sonar_found && odo_found)
            {
                break;
            }
        }
    }

    float sonar_height = sonar_msg->range;
    geometry_msgs::msg::Point odo_ = odo_msg->pose.pose.position;

    geometry_msgs::msg::Point goal_2;
    goal_2.set__x(2);
    goal_2.set__y(5);
    goal_2.set__z(0.5);

    float height = sonar_height - odo_.z;
    float distanceDiff_2;

    distanceDiff_2 = sqrt(pow(goal_2.x - odo_.x, 2) + pow(goal_2.y - odo_.y, 2));

    DataEvaluator dataEvaluator;

    float gradient2 = dataEvaluator.evaluateGradient(height, goal_2.z, distanceDiff_2);

    std::cout << "Calculated Gradient: "<< gradient2 << std::endl;

    EXPECT_FALSE(gradient2 < 0.03);
}


