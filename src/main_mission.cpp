#include "rclcpp/rclcpp.hpp"
#include "mission.h"
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission>());
  rclcpp::shutdown();
  return 0;
}
