# The Grand Tour Project

## Introduction

41012 Programming for Mechatronic Systems — Autumn 2024

**Student Name:** Minh Nguyen
**Student Number:** 13582453
**Student Email:** Minh-K-Nguyen-3@student.uts.edu.au

The Grand Tour Project involves the sensing and control of Ackerman and Quadcopter platforms, aiming to navigate the Ackerman through rough terrain with the assistance of the Quadcopter. The project focuses on autonomous control, sensor integration, and ROS2 communication.

> [!WARNING]
> Sometimes the mission node can't get goals from `/mission/goals` for some reason. Just keep sending them until the mission node says something like `[mission]: Have: 2 goals`.
> One way to avoid this is to run the mission node first, supply goals (or use the `Sample_goals_publisher`), **then** run the quadcopter and ackerman nodes.

Also note: I did my best with advanced mission, but it's not perfect. It works for the most part, but sometimes it doesn't.

### Mission Running

[![Mission Running Demo](https://img.youtube.com/vi/_HBZnPkieTs/0.jpg)](https://youtu.be/_HBZnPkieTs)

### TSP

![TSP](docs/images/tsp.png)
<!-- Update this path to wherever tsp.png actually lives in your repo -->

## Installation and Running

### Step 1: Clone the repository

```sh
git clone https://github.com/41012/pfms-2024a-FattDuckk
```

### Step 2: Link to ROS2 workspace and build the project

Link and build the project in the ROS2 workspace. To build, go to the ROS2 workspace and run the following command after linking the project to the source folder:

```sh
colcon build --symlink-install --packages-select a3_submission
source install/setup.bash
```

### Step 3: Launch ROS2 gazebo_tf simulation

In a terminal, run:

```sh
ros2 launch gazebo_tf a3_project3.launch.py
```

### Step 4: Launch project nodes

In three different terminals, run one of the following commands each to launch the project nodes:

```sh
ros2 run a3_submission mission
```

```sh
ros2 run a3_submission quadcopter
```

```sh
ros2 run a3_submission ackerman
```

### Step 5: Example moving command

Change `"orange"` to `"drone"` to move the quadcopter to the desired position.

```sh
ros2 service call /demo/set_entity_state 'gazebo_msgs/srv/SetEntityState' '{state: {name: "orange", pose: {position: {x: -11, y: -5, z: 0.2}}}}'
```

### Step 6: Example supplying goals

Change `"orange"` to `"drone"` to move the quadcopter to the desired position. Note that in advanced mode, the last goal is assumed to be the final destination.

```sh
ros2 topic pub -1 /mission/goals geometry_msgs/msg/PoseArray "{
poses: [
{position: {x: -3.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: -2.5, y: 4.5, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
]
}"
```

### Step 7: Running the mission

```sh
ros2 service call /mission/control std_srvs/srv/SetBool "{data: true}"
```

### Step 8: Running the advanced mission

To run the advanced mission instead of `ros2 run a3_submission mission`:

```sh
ros2 run a3_submission mission --ros-args -p _advanced:=true
```

## Classes Overview

### Ackerman Class

The `Ackerman` class represents an Ackerman-steered vehicle, providing functionalities such as goal checking, velocity control, and odometry updates.

This node also uses the `audi` library for steering calculation. It subscribes to `/orange/laserscan` to detect obstacles in front of the Ackerman and immediately stops when an obstacle is detected within 1m.

`Ackerman::updateBrakeInfo()` uses the mass, velocity, and a default brake force to calculate the brake force needed to stop the Ackerman at any speed — allowing the car to move at high speed and stop accurately at the goal. This was the first function I wrote on my own for this subject, and I'm very proud of it.

**Ackerman Methods**
- `Ackerman()` — Constructor for the Ackerman class.
- `~Ackerman()` — Destructor for the Ackerman class.
- `bool reachGoal()` — Checks if the goal has been reached.
- `bool calcNewGoal()` — Calculates whether the Ackerman needs to start braking.
- `bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal, double &distance, double &time, geometry_msgs::msg::Pose &estimatedGoalPose)` — Checks the distance and time from origin to destination.
- `void velocityCallback(const geometry_msgs::msg::TwistStamped &msg)` — Callback function for velocity messages.
- `bool updateBrakeInfo()` — Updates brake information.
- `void sendAckCmd(double brake, double steering, double throttle)` — Sends control commands to the Ackerman.
- `void odoCallback(const nav_msgs::msg::Odometry &msg)` — Callback function for odometry messages.

### Quadcopter Class

The `Quadcopter` class represents a quadcopter drone, providing functionalities such as goal checking, control commands, and odometry updates.

It subscribes to `/drone/laserscan` to detect obstacles in front of it and immediately stops when an obstacle is detected within 0.3m.

**Quadcopter Methods**
- `Quadcopter()` — Constructor for the Quadcopter class.
- `~Quadcopter()` — Destructor for the Quadcopter class.
- `bool reachGoal()` — Checks if the Quadcopter has reached its goal.
- `bool calcNewGoal()` — Calculates the angle needed for the Quadcopter to reach a goal.
- `bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal, double &distance, double &time, geometry_msgs::msg::Pose &estimatedGoalPose)` — Checks the distance and time from origin to destination.
- `void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)` — Callback function for the control service.
- `void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b)` — Sends control commands to the Quadcopter.
- `void sendTakeOff()` — Sends takeoff command to the Quadcopter.
- `void sendLanding()` — Sends landing command to the Quadcopter.

### Mission Class

The `Mission` class represents the overall mission control, managing goals, progress, and state updates for both the Ackerman and Quadcopter.

In advanced mode, it sorts through an array of poses to determine the TSP path for the drone to follow.

**Mission Methods**
- `Mission()` — Constructor for the Mission class.
- `~Mission()` — Destructor for the Mission class.
- `void cmd_drone_goal(geometry_msgs::msg::Point goal)` — Sets the goal for the drone.
- `void cmd_orange_goal(geometry_msgs::msg::Point goal)` — Sets the goal for the Ackerman ("orange").
- `void run()` — Function that runs continuously in a separate thread.
- `void progress()` — Updates the progress of the mission.
- `double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt)` — Calculates the distance between two points.
- `void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)` — Callback function for receiving goal pose messages.
- `void odomCallback_drone(const std::shared_ptr<nav_msgs::msg::Odometry> msg)` — Callback function for receiving drone odometry messages.
- `void odomCallback_orange(const std::shared_ptr<nav_msgs::msg::Odometry> msg)` — Callback function for receiving Ackerman odometry messages.
- `void sonarCallback_(const std::shared_ptr<sensor_msgs::msg::Range> msg)` — Callback function for receiving sonar range messages.
- `std::vector<geometry_msgs::msg::Point> generateIntermediateWaypoints(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal, double interval)` — Generates intermediate waypoints between two points.
- `void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)` — Callback function for controlling the mission.

### Controller Class

The `Controller` class serves as a base class for platform controllers, providing shared functionalities and interfaces for platform-specific controllers like Ackerman and Quadcopter.

**Controller Methods**
- `Controller()` — Default constructor for the Controller class.
- `virtual bool calcNewGoal() = 0` — Instructs the underlying platform to recalculate a goal.
- `void setGoal(const geometry_msgs::msg::Point &msg)` — Sets the goal for the platform.
- `void laserCallback(const sensor_msgs::msg::LaserScan &msg)` — Callback function for laser scan data.
- `virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin, geometry_msgs::msg::Point goal, double &distance, double &time, geometry_msgs::msg::Pose &estimatedGoalPose) = 0` — Checks whether the platform can travel between origin and destination.
- `bool setTolerance(double tolerance)` — Sets the tolerance radius for reaching the goal.
- `double distanceTravelled()` — Returns the total distance travelled by the platform.
- `double timeInMotion()` — Returns the total time spent in motion by the platform.
- `double distanceToGoal()` — Returns the current distance to the goal.
- `double timeToGoal()` — Returns the estimated time to reach the goal.
- `void updateDistanceTravelled()` — Updates the total distance travelled by the platform.
- `void updateTimeInMotion()` — Updates the total time spent in motion by the platform.
- `geometry_msgs::msg::Pose getOdometry()` — Returns the current odometry of the platform.
- `virtual void odoCallback(const nav_msgs::msg::Odometry &msg)` — Callback function for odometry data.

## Acknowledgements

Special thanks to Alen Alempijevic for his guidance and support throughout the project, and for an amazing and super stressful semester. This project was developed as part of the robotics systems course, focusing on the integration of ROS2, sensor data processing, and autonomous control.

## Contact

For more information, contact me at Minh.K.Nguyen-3@student.uts.edu.au.
