# Navigate ROS 2 Package

The `navigate` ROS 2 package provides nodes and utilities for controlling a differential drive robot's navigation behavior. This package is made as a learning exercise in ROS2 humble to learn how to use topics, actions, parameters, and how to link multiple nodes together.

## Nodes

### 1. NavController

The `NavController` is responsible for controlling the linear and angular velocities of a differential drive robot based on a reference pose received from a topic. It implements two PID controllers to compute the required velocities for navigation.

#### Subscribed Topics

- `/input/pose` (geometry_msgs/PoseStamped): Receives the desired reference pose (position and orientation) for navigation.
- `/input/odom` (nav_msgs/Odometry): Receives the odometry data of the robot, including its current position and orientation.

#### Published Topics

- `/output/cmd_vel` (geometry_msgs/TwistStamped): Publishes the computed linear and angular velocities as TwistStamped messages to control the robot's motion.

#### Parameters

- `kp_linear`: Proportional gain for linear velocity control.
- `ki_linear`: Integral gain for linear velocity control.
- `kd_linear`: Derivative gain for linear velocity control.
- `kp_angular`: Proportional gain for angular velocity control.
- `ki_angular`: Integral gain for angular velocity control.
- `kd_angular`: Derivative gain for angular velocity control.
- `max_vel_linear`: Maximum linear velocity allowed for the robot.
- `max_vel_angular`: Maximum angular velocity allowed for the robot.
- `tolerance`: Tolerance for position error, used for angle reference when the robot is close to the desired position.

### 2. NavPoseServer

The `NavPoseServer` is an action server node that provides the functionality for navigating a differential drive robot to a specified pose. It listens for goal requests and cancel requests, tracks the current vehicle odometry, and executes the navigation action based on the received goals.

#### Action

- `nav_pose` (navigate_msgs/NavPose): Defines the navigation action, which includes a goal pose and provides feedback on the navigation progress.

#### Parameters

- `position_tolerance`: Tolerance for position error, used to determine when the goal position is reached.
- `yaw_tolerance`: Tolerance for orientation error (yaw), used to determine when the goal orientation is reached.
- `timeout`: Maximum duration allowed for completing the navigation action.

### 3. NavPoseClient

The NavPoseClient node allows external clients to send navigation goal requests to the nav_pose_server action server. It sends the desired goal pose to the server and monitors the navigation progress.

## Usage

To use the `navigate` package, you can launch the nodes individually or include them in your own launch files. Make sure to configure the parameters according to your robot's specifications and navigation requirements.

```bash
ros2 launch navigate launch_nav.launch.py
```

## Compatibility

This package is designed to work with differential drive robots and has been tested on ROS 2 Foxy and later versions.