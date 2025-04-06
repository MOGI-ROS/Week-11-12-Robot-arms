[//]: # (Image References)

[image1]: ./assets/starter-package.png "Starter package"
[image2]: ./assets/starter-package-1.png "Starter package"
[image3]: ./assets/openmanipulator.png "OpenMANIPULATOR-X"
[image4]: ./assets/om.png "OpenMANIPULATOR-X in Gazebo"
[image5]: ./assets/om-1.png "OpenMANIPULATOR-X in Gazebo"
[image6]: ./assets/om-2.png "OpenMANIPULATOR-X with MoveIt"

# Week 11-12: Robot arms
Simulations of OpenMANIPULATOR-X and UR3e using MoveIt 2

## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/VNM-FqoVrW8"><img width="600" src="./assets/youtube.png"></a>  

# Table of Contents
1. [Introduction](#introduction)  
1.1. [Download ROS package](#download-ros-package)  
2. [OpenMANIPULATOR-X](#openmanipulator-x)  
2.1. [Packages needed](#packages-needed)  
2.2. [First try](#first-try)  
2.3. [MoveIt](#MoveIt)  
2.4. [Writing our own launch files](#writing-our-own-launch-files)  
2.5. [Placing it into our own world](#placing-it-into-our-own-world)  
2.6. [Adding a gripper camera](#adding-a-gripper-camera)  
2.7. [Custom nodes for moving the robot](#custom-nodes-for-moving-the-robot)  
2.8. [Inverse kinematics](#inverse-kinematics)  
3. [Unviersal Robots](#universal-robots)  
3.1. [First try of the UR3e](#first-try-of-the-ur3e)  
3.2. [UR3e with MoveIt](#ur3e-with-moveit)  
3.3. [Adding a gripper](#adding-a-gripper)  
3.4. [Updating MoveIt config](#updating-moveit-config)  
3.5. [Gazebo world](#gazebo-world)  
3.6. [MoveIt commander](#moveit-commander)  

# Introduction

In this lesson we'll lear how to use off the shelf robotic arms, make them move with our own ROS2 nodes and using MoveIt 2!

## Download ROS package

To download the starter package, clone the following git repo with the `starter-branch` (using the `-b branch` flag) into your colcon workspace:
```bash
git clone -b starter-branch https://github.com/MOGI-ROS/Week-11-12-Robot-arms
```

Let's take a look what's inside the package with the `tree` command!

TODO:

# OpenMANIPULATOR-X

The first robotic arm we try is the OpenMANIPULATOR-X from Dynamixel. It's a simple 4 DoF robotic arm with a gripper that cannot be changed, it's similar to the simple robotic arm we did in the previous lesson.

![alt text][image3]

It has a very limited space to reach with a maximum 500g payload capability. Although it's small, it's very well suidatble for educational purposes, it's also available at the MOGI faculty. The official documentation can be found [here](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_operation/).


## Packages needed

We will need a few official ROS packages to control the gripper of the simulated OpenMANIPULATOR-X:

```bash
sudo apt install ros-jazzy-gripper-controllers
sudo apt install ros-jazzy-position-controllers
```

The other package we need is available on the MOGI-ROS organization on GitHub, this is a fork from the official Dynamixel repo, clone it to your workspace and rebuild it:

```bash
git clone -b jazzy https://github.com/MOGI-ROS/open_manipulator
```

> Which is equivalent with 3.1.0 tag in [the official repo](https://github.com/ROBOTIS-GIT/open_manipulator).

## First try

Once we have all the necessary packages, we have to set up the `ROBOT_MODEL` environmental variable to `om_x` to use the packages for OpenMANIPULATOR-X, the best solution is still setting it up in the `.bashrc` file:

```bash
export ROBOT_MODEL=om_x
```

To start the simulation we have to start the following launch file:
```bash
ros2 launch open_manipulator_bringup gazebo.launch.py
```

> If you have to switch to the old OGRE rendering engine, you can do that in the `gazebo.launch.py` file
> ```python
>    gazebo = IncludeLaunchDescription(
>                 PythonLaunchDescriptionSource([os.path.join(
>                     get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
>                 launch_arguments=[
>                     ('gz_args', [LaunchConfiguration('world'),
>                                  '.sdf',
>                                  ' -v 1',
>                                  ' -r',
>                                  ' --render-engine ogre --render-engine-gui-api-backend opengl'
>                                  ]
>                     )
>                 ]
>              )
> ```

And then we can control the robot with `rqt_joint_trajectory_controller` that we used earlier:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

![alt text][image4]

As you can see, it's not possible to adjust the gripper from the `rqt_joint_trajectory_controller` because it's not a `JointTrajectoryController` as we used in the previous lesson, but it's a `GripperActionController`. `GripperActionController` cannot be controlled by the `rqt_joint_trajectory_controller`.

---

Let's see how can we control the gripper then? Start the simulation as before:

```bash
ros2 launch open_manipulator_bringup gazebo.launch.py
```

And in another terminal start the following node:
```bash
ros2 run open_manipulator_teleop keyboard_control_x.py
```

We can control the gripper with the `o` and `p` keys.

![alt text][image5]

## MoveIt

OpenMANIPULATOR-X works with MoveIt 2 out of the box, we just have to start the simulation as always:

```bash
ros2 launch open_manipulator_bringup gazebo.launch.py
```

And in another terminal start the following launch file:

```bash
ros2 launch open_manipulator_moveit_config moveit_core.launch.py
```

![alt text][image6]



---

ros2 launch open_manipulator_bringup gazebo.launch.py
ros2 launch open_manipulator_moveit_config move_group.launch.py
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py


## Writing our own launch files 

ros2 launch open_manipulator_mogi simulation_bringup.launch.py

## Placing it into our own world


## Adding a gripper camera

git clone -b mogi_jazzy https://github.com/MOGI-ROS/open_manipulator

## Custom nodes for moving the robot


## Inverse kinematics

IK, set red box to 25cm








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


SRDF file add new home