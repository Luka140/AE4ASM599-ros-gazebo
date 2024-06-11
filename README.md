# ROS2 and Gazebo
This project documents how to use Ros2 Humble in conjunction with Gazebo Ignition.

## Stetting up the Development Environment.
A standardised development environment is achieved through the use of devcontainers with vscode. For ubuntu based systems, the following system dependencies are required.
* **vscode**, setup for [dev containers](https://code.visualstudio.com/docs/devcontainers/containers)
* [**docker engine**](https://docs.docker.com/engine/install/ubuntu/)

Clone the git repositry and open it in vscode, a prompt will appear asking to open the project in the dev container. Alternatively, The project can be opened in the dev container by running ```Dev Containers: rebuild and reopen in container``` in the vscode command palette.


## Pathfollowing
The simulation can be started using:
ros2 launch my_bot launch_sim.launch.py
 
The action server using:
ros2 launch navigate test_params.launch.py
 
Then the action client can be ran using:
ros2 run navigate nav_pose_client


# capita_selecta
