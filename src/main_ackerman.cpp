#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "ackerman.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ackerman>());
  rclcpp::shutdown();

  return 0;
}
