# Pathfollowing
Package written by Luka Distelbrink

This package was the first package I wrote after doing the Gazebo tutorials. It was just an exercise to practice using ROS2 and Gazebo. 

This contains a node that performs simple waypoint following missions while avoiding obstacles. The algorithm is not robust and the waypoints may not be accurate to the simulated world due to error buildup in odometry.

## Usage 
Open the gazebo file using:
```
ign gazebo gazebo_files/worlds/pathfollowing.sdf
```

In a new window build and launch the nodes:
```
colcon build --packages-select pathfollowing --merge-install --symlink-install
```

```
source install/setup.bash
ros2 launch pathfollowing launch_pathfinding.launch.py
```

Then in a third window, a waypoint can be set using the following command:
```
ros2 topic pub --once waypoint geometry_msgs/msg/Pose "{position: {x: 10, y: 0.0, z: 0.0}}"
```

## Pathfinder
This node sends velocity commands to drive the vehicle to a set waypoint, avoiding obstacles using lidar data. 


#### Subscribers

- Position Subscriber

        Type: Odometry messages
        Topic: /model/vehicle_blue/odometry
        Callback: movement_cmd
        Purpose: Receives the current position of the vehicle. This information is used to calculate movement commands towards the objective position.

- Objective Subscriber
  
        Type: Pose messages
        Topic: waypoint
        Callback: set_objective
        Purpose: Receives the objective position (waypoint) where the vehicle needs to navigate. This position is set as the destination for the vehicle.

- Lidar Subscriber
  
        Type: LaserScan messages
        Topic: lidar
        Callback: set_lidar
        Purpose: Receives laser scan data from a LiDAR sensor. This data is used to detect obstacles and adjust the vehicle's navigation path accordingly.

#### Publishers

- Velocity Command Publisher
  
        Type: Twist messages
        Topic: cmd_vel
        Purpose: Publishes velocity commands to control the movement of the vehicle based on the calculated navigation commands.

