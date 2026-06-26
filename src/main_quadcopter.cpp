#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Quadcopter>());
  rclcpp::shutdown();

  return 0;
}
