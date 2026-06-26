// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include the header file for the SetEntityState service
#include "gazebo_msgs/srv/set_entity_state.hpp"

// We state the test suite name and the test name
TEST(AckermanTest, SetEntityState){

    // Initialize the ROS2 node
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("ackerman_test_node");

    // Create a client for the SetEntityState service
    auto client = node->create_client<gazebo_msgs::srv::SetEntityState>("/demo/set_entity_state");

    // Wait for the service to be available
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    // Create a request to set the entity state
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = "orange";
    request->state.pose.position.x = -11;
    request->state.pose.position.y = -5;
    request->state.pose.position.z = 0.2;

    // Send the request
    auto result = client->async_send_request(request);

    // Wait for the result and check if the service call was successful
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Successfully set entity state");
        // Add your assertions here if needed
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to set entity state");
        FAIL() << "Service call failed";
    }

    // Shutdown ROS2
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
