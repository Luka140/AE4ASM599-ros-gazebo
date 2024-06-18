# ROS2 and Gazebo

ROS2 (Robot Operating System 2) and Gazebo are a popular toolbase for robotics development, offering a flexible and scalable framework for building and simulating robotic systems. ROS2 provides a modular architecture, real-time capabilities, and security features, while Gazebo offers a realistic 3D simulation environment for testing and validating robotic systems. Together, they enable developers to design, test, and deploy robotic systems efficiently and effectively. The ROS2 and Gazebo toolbase is widely used in robotics research, industrial automation, and autonomous systems, and is supported by a large and active community.

## Project Purpose

The primary goal of this project is to learn how to effectively use ROS2 and Gazebo, and to document the process and findings. By exploring the functionalities and capabilities of these tools, we aim to gain a deeper understanding of robotic system development and simulation. This project aims to provide valuable insights and practical guidance on leveraging ROS2 and Gazebo for your robotics applications.

## Project Description
A differential drive rover equipped with odometry and lidar sensors is simulated in gazebo to demonstrate the processing of lidar data to an occupancy grid and the capabilities to navigate to a desired pose.
This project consists of the following packages:

- **simulation**: Simulating a Differential Drive Robot in Gazebo: This package includes the necessary configuration and launch files to simulate a differential drive robot within Gazebo. It sets up the robot model, sensors, and environment to create a realistic simulation for testing and development purposes.

- **navigate**: Navigation Package for Pose Control of the Robot: This package provides the functionality to navigate the robot to a desired position and orientation within the simulated environment. It includes algorithms and controllers to handle path planning, obstacle avoidance, and precise maneuvering to achieve the specified pose goals.

- **pointcloud_to_occupancy_grid**: This package processes the lidar sensor data collected from the robot and converts it into an occupancy grid. This grid represents the environment in a format suitable for navigation and mapping tasks, facilitating further processing and decision-making by the robot's control system.

# Setup
As an alternative to manually installing ROS2 Humble and Gazebo Fortress, you can use a Docker container to set up your development environment. This approach simplifies the setup process and ensures a consistent environment. This guide will walk you through the steps to use a Dockerfile in Visual Studio Code (VS Code) with the Dev Containers extension.

## Prerequisites
Before you begin, ensure you have the following installed on your system:

* [Docker](https://docs.docker.com/get-docker/)
* [Visual Studio Code (VS Code)](https://code.visualstudio.com/)
* [Dev Containers extension for VS Code](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Installation Instructions
Download the [`.devcontainer`](../tree/main/.devcontainer) directory, which contains the `Dockerfile` and `devcontainer.json` file. This directory should be placed at the root of your project.

```
.devcontainer/
├── devcontainer.json
└── Dockerfile
```
Next open up the project in VS Code and build the devcontainer environment by opening the Command Palette in VS Code (**'Ctrl+Shift+P'**) and running `Rebuild and Reopen in Container`. This process might take a few minutes the first time.

