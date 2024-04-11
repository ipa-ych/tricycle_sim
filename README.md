# tricycle_sim
Simulation of cob_base and minimum_tricycle_model using tricycle_controller and tricycle_steering_controller in gazebo 11 classic

## Prerequisite
1. ROS2 (Tested on Humble)
2. Gazebo 11 classic
3. ROS2 Control & gazebo_ros2_control ([Tricycle_drive_controller](https://control.ros.org/master/doc/ros2_controllers/tricycle_controller/doc/userdoc.html) and [Tricycle_steering_controller](https://control.ros.org/master/doc/ros2_controllers/tricycle_steering_controller/doc/userdoc.html))
4. RQT
5. [Pointcloud to Laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)
6. [ros2_laser_scan_merger](https://github.com/ipa-ych/ros2_laser_scan_merger.git)

## How to use 
1. Clone repos in Prerequisite list
2. Build package and source
```bash
colcon build --symlink-install
source install/setup.bash
```
3. Launch demo
- cob_teleop using joystick with tricycle_steering_controller
```bash
ros2 launch tricycle_sim cob_steering_0408.launch.py
```
- cob_slam using slam_toolbox with tricycle_drive_controller
```bash
ros2 launch tricycle_sim cob_drive_0411.launch.py world:= < path_to_world >
```