# Week-11-12-Robot-arms
Simulations of OpenMANIPULATOR-X and UR3e using MoveIt 2


# Tartalomjegyzék
1. [Kezdőcsomag](#Kezdőcsomag)  
2. [OpenMANIPULATOR-X](#OpenMANIPULATOR-X)  
2.1. [Első próba](#Első-próba)  
2.2. [Indítás MoveIt-tel](#Indítás-MoveIt-tel)  
2.3. [IKFast plugin](#IKFast-plugin)  
2.4. [Saját node mozgatáshoz](#Saját-node-mozgatáshoz)  
2.5. [Gazebo világba helyezés](#Gazebo-világba-helyezés)  
2.6. [Saját IK](#Saját-IK)  
3. [UR3e robotkar](#UR3e-robotkar)  
3.1. [UR3e gripperrel](#UR3e-gripperrel)  
3.2. [Gazebo világ](#Gazebo-világ)  
3.3. [SRDF fájl](#SRDF-fájl)  
3.4. [MoveIt commander](#MoveIt-commander)  

sudo apt install ros-jazzy-gripper-controllers
sudo apt install ros-jazzy-position-controllers


https://github.com/MOGI-ROS/open_manipulator Jazzy branch

.bashrc
export ROBOT_MODEL=om_x
ros2 launch open_manipulator_bringup gazebo.launch.py

rqt_joint_trajectory_controller

---

ros2 run open_manipulator_teleop keyboard_control_x.py

---

ros2 launch open_manipulator_moveit_config moveit_core.launch.py

---

ros2 launch open_manipulator_moveit_config move_group.launch.py
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py



---


https://github.com/MOGI-ROS/Universal_Robots_ROS2_GZ_Simulation

https://github.com/ros-industrial/ur_msgs humble
https://github.com/UniversalRobots/Universal_Robots_Client_Library
https://github.com/MOGI-ROS/Universal_Robots_ROS2_Description
https://github.com/MOGI-ROS/Universal_Robots_ROS2_Driver


ros2 launch ur_simulation_gz ur_sim_control.launch.py
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller

---

ros2 launch ur_simulation_gz ur_sim_moveit.launch.py