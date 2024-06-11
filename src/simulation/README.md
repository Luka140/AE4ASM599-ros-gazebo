## Simulation Package
This package provides a simulation environment containing the gazebo world and robot description. By bundeling this together with a convenient launch file the workflow for developing robotics systems is significantly simplified.

### Utilisation
Launch the sim environment:
```bash
ros2 launch simulation launch_sim.launch.py
```
This will start gazebo, spawn the robot and initialise the controller manager and its controllers.
Rviz is launched by default, this can be disabled by setting the `gui:=false`.

### Package structure
The structure of the package is as follows.
```
📦sim
 ┣ 📂config
 ┣ 📂description
 ┣ 📂launch
 ┣ 📂worlds
 ┣ 📜CMakeLists.txt
 ┗ 📜package.xml
 ```
 
#### Robot Description
The robot description is provided in the **description** directory in `URDF` format. `xacro` is used to allow for a parametrised description of the robot.
* **robot_core.xacro:** describes the links and joints making up the robot. Defines the physical layout of the robot.
* **ros2_control.xacro:** contains the ros2 control setup of the robot. Defines the behaviour of the actuators and exposes these to ROS2.
* **lidar.xacro:** contains lidar sensor setup.

The robot description is fully parametrised and all parameters are located in **robot.urdf.xacro**.

The controllers for ros2 control are specified in the config file **controllers.yaml**

#### Worlds
The gazebo world files are provided in the **worlds** directory in the `sdf` format. the desired world can then be selected in the launch file.

#### Launch File
All the parts of the simulationi package are tied together in the launch files. 

### Dependencies
The following packages and tools are required (in adittion to ROS2 humble):
* [**Gazebo Fortress (Igntion):**](https://gazebosim.org/docs/fortress/install) Is provided as a package and can be installed as `ros-humble-ros-gz`.
* [**XACRO:**](http://wiki.ros.org/xacro) Is provided as a package and can be installed as `ros-humble-xacro`
* [**Ros2 control**](https://control.ros.org/rolling/index.html) Is provided as a package and can be installed as `ros-humble-ign-ros2-control`
