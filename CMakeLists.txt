cmake_minimum_required(VERSION 3.5)
project(a3_submission)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(gazebo_msgs REQUIRED)
#################################################################


add_library(controllers src/controller.cpp src/quadcopter.cpp src/ackerman.cpp)

ament_target_dependencies(controllers rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

# The library we are developing will be called laserprocessing
# add_library(laserprocessing src/laserprocessing.cpp)
# ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs)

# ## This is the executable
# add_executable(sample src/main.cpp src/sample.cpp)


## TASK 5 - Add the package to the end of the dependencies
# ament_target_dependencies(sample rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)

# target_link_libraries(sample laserprocessing controllers)

#################################################################
add_executable(ackerman src/main_ackerman.cpp src/controller.cpp src/ackerman.cpp)
ament_target_dependencies(ackerman rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs )

#################################################################
add_executable(quadcopter src/main_quadcopter.cpp src/controller.cpp src/quadcopter.cpp)
ament_target_dependencies(quadcopter rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs )

#################################################################
add_executable(mission src/main_mission.cpp src/mission.cpp src/tsp_search.cpp src/dataEvaluation.cpp)
#ament_target_dependencies(platform rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(mission rclcpp std_msgs sensor_msgs nav_msgs tf2 visualization_msgs std_srvs)

add_executable(Sample_goals_publisher src/goals_publisher.cpp )
ament_target_dependencies(Sample_goals_publisher rclcpp geometry_msgs visualization_msgs)

target_link_libraries(ackerman audi)


install(TARGETS
  mission
  ackerman
  quadcopter
  Sample_goals_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)

# target_link_libraries(test_laserprocessing laserprocessing)
# target_link_libraries()
  ament_add_ros_isolated_gtest(testWaypoint test/test_waypoint.cpp src/dataEvaluation.cpp)
  ament_target_dependencies(testWaypoint rclcpp sensor_msgs nav_msgs rosbag2_cpp geometry_msgs)

  ament_add_ros_isolated_gtest(testGoal test/test_goal.cpp src/dataEvaluation.cpp)
  ament_target_dependencies(testGoal rclcpp sensor_msgs nav_msgs rosbag2_cpp geometry_msgs)

  ament_add_ros_isolated_gtest(testTSP test/test_tsp.cpp src/tsp_search.cpp)
  ament_target_dependencies(testTSP rclcpp sensor_msgs nav_msgs rosbag2_cpp geometry_msgs)

endif()

ament_package()
