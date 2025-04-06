[//]: # (Image References)

[image1]: ./assets/starter-package.png "Starter package"
[image2]: ./assets/starter-package-1.png "Starter package"
[image3]: ./assets/openmanipulator.png "OpenMANIPULATOR-X"
[image4]: ./assets/om.png "OpenMANIPULATOR-X in Gazebo"
[image5]: ./assets/om-1.png "OpenMANIPULATOR-X in Gazebo"
[image6]: ./assets/om-2.png "OpenMANIPULATOR-X with MoveIt"
[image7]: ./assets/om-3.png "OpenMANIPULATOR-X with GUI"
[image8]: ./assets/om-4.png "OpenMANIPULATOR-X launch"
[image9]: ./assets/om-5.png "OpenMANIPULATOR-X Gazebo world"
[image10]: ./assets/om-6.png "OpenMANIPULATOR-X with camera"
[image11]: ./assets/om-7.png "OpenMANIPULATOR-X open gripper"
[image12]: ./assets/om-8.png "OpenMANIPULATOR-X joint angles"
[image13]: ./assets/om-9.png "OpenMANIPULATOR-X close gripper"
[image14]: ./assets/ik.png "OpenMANIPULATOR-X IK"
[image15]: ./assets/om-10.png "OpenMANIPULATOR-X red box"
[image16]: ./assets/om-11.png "OpenMANIPULATOR-X open gripper"
[image17]: ./assets/om-12.png "OpenMANIPULATOR-X IK angles"

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

There is another GUI made by Dynamixel that uses MoveIt in the background. Start the simulation as usually:
```bash
ros2 launch open_manipulator_bringup gazebo.launch.py
```

Start MoveIt without RViz:
```bash
ros2 launch open_manipulator_moveit_config move_group.launch.py
```

And finally start the GUI:
```bash
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py
```

![alt text][image7]

## Writing our own launch files 

We can write our own launch files for bringup, like the `simulation_bringup.launch.py` in the `open_manipulator_mogi` package. With this launch file we have more flexibility, we can later easily spawn the robot in our own world and we can also adjust the coordinates where it spawns.

```bash
ros2 launch open_manipulator_mogi simulation_bringup.launch.py
```

The launch file will start the simulation and it also opens RViz.

![alt text][image8]

---

There is another launch file the `simulation_moveit_bringup.launch.py` which initialize the simulation like before and starts MoveIt with RViz too. We can notice that MoveIt is started only after the controllers are loaded. If it's starting before every controller is loaded it might cause errors during startup.

```python
    # Launch rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open_manipulator_moveit_config, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open_manipulator_moveit_config, 'launch', 'move_group.launch.py')
        )
    )

    # Load MoveIt! only after controllers are loaded
    move_it_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[move_group_launch,
                        rviz_launch],
            )
    )
```

## Placing it into our own world

With the new launch files it's very easy to load the robot in our own world:

```bash
ros2 launch open_manipulator_mogi simulation_bringup.launch.py world:=world.sdf z:=1.02
```

![alt text][image9]

> We can change the default values of world and the z coordinate in the launch file.

## Adding a gripper camera

Let's add a gripper camera, as we did before we have to add a `gripper_camera_link` and a `gripper_camera_link_optical` with the necessary joints. The camera_link's parent is `link5`.

Edit the follwoing file URDF file first: `/open_manipulator/open_manipulator_description/urdf/om_x/open_manipulator_x_arm.urdf.xacro`

And add the camera related links and joints to the end of the file:
```xml
    ...

    <!-- Gripper camera -->
    <joint type="fixed" name="${prefix}gripper_camera_joint">
      <origin xyz="0.08 0.0 0.0" rpy="0 0 0"/>
      <child link="${prefix}gripper_camera_link"/>
      <parent link="${prefix}link5"/>
    </joint>

    <link name='${prefix}gripper_camera_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass value="1.0e-03"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia
            ixx="1e-6" ixy="0" ixz="0"
            iyy="1e-6" iyz="0"
            izz="1e-6"
        />
      </inertial>

      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size=".01 .01 .01"/>
        </geometry>
        <material name="red"/>
      </visual>
    </link>

    <joint type="fixed" name="${prefix}gripper_camera_optical_joint">
      <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
      <child link="${prefix}gripper_camera_link_optical"/>
      <parent link="${prefix}gripper_camera_link"/>
    </joint>

    <link name="${prefix}gripper_camera_link_optical">
    </link>

  </xacro:macro>

</robot>
```

Next, we have to add the Gazebo plugin, edit the following file: `/open_manipulator/open_manipulator_description/gazebo/open_manipulator_x.gazebo.xacro`

And add the plugin to the end of the file:
```xml
  ...

  <gazebo reference="${prefix}gripper_camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.01</near>
          <far>15</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <optical_frame_id>${prefix}gripper_camera_link_optical</optical_frame_id>
        <camera_info_topic>${prefix}gripper_camera/camera_info</camera_info_topic>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>${prefix}gripper_camera/image</topic>
    </sensor>
  </gazebo>

  </xacro:macro>

</robot>
```

> These changes are already available on the `mogi-jazzy` branch of the `open_manipulator` repository. You can download this branch if it's not downloaded yet:
> ```bash
> git clone -b mogi-jazzy https://github.com/MOGI-ROS/open_manipulator
> ```
> or if you already cloned it, you can switch branches with a graphical GIT client or with the following CLI command (in the folder of `open_manipulator`):
> ```bash
> git switch mogi-jazzy
> ```

We still have to add a few nodes to our `simulation_bringup.launch.py` file:

```python
    # Node to bridge camera topics
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/gripper_camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'gripper_camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay node to republish camera_info to image/camera_info
    relay_gripper_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['gripper_camera/camera_info', 'gripper_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Also add them to the launch description object:

```python
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_gripper_camera_info_node)
```

And finally we have to add the `camera_info` topic to our `gz_bridge.yaml` config file:

```yaml
- ros_topic_name: "gripper_camera/camera_info"
  gz_topic_name: "gripper_camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: "GZ_TO_ROS"
```

Rebuild the workspace and then we are ready to start the simulation:

```bash
ros2 launch open_manipulator_mogi simulation_bringup.launch.py
```

![alt text][image10]

## Custom nodes for moving the robot

There are 3 nodes in the `open_manipulator_mogi_py` package:

1. `send_joint_angles.py`: send joint angles on the `/arm_controller/joint_trajectory` topic.
2. `open_gripper.py`: an action client to send `open` (`0.019`) gripper position through the `/gripper_controller/gripper_cmd` action.
3. `close_gripper.py`: an action client to send `closed` (`0.0`) gripper position through the `/gripper_controller/gripper_cmd` action.
4. `inverse_kinematics.py`: similar inverse kinematics implementation that we used in the previous lesson. We'll use this in the next chapter.

Start the simulation:  
```bash
ros2 launch open_manipulator_mogi simulation_bringup.launch.py
```


And try opening the gripper first:
```bash
ros2 run open_manipulator_mogi_py open_gripper
```
![alt text][image11]

Then send the joint angles:
```bash
ros2 run open_manipulator_mogi_py send_joint_angles
```
![alt text][image12]

Start the `rqt_joint_trajectory_controller` in a new terminal and close the gripper. We can now lift and move the red box.

And try opening the gripper first:
```bash
ros2 run open_manipulator_mogi_py close_gripper
```
![alt text][image13]


## Inverse kinematics

The links and joints of OpenMANIPULATOR-X are very similar to the simple robotic arm that we built during the previous lesson. With some small modifications we can use the same geometric inverse kinematic solver script with it.  
![alt text][image14]

```python
def inverse_kinematics(self, coords, gripper_angle = 0):
    '''
    Calculates the joint angles according to the desired TCP coordinate and gripper angle
    :param coords: list, desired [X, Y, Z] TCP coordinates
    :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
    :return: list, the list of joint angles, including the 2 gripper fingers
    '''
    # link lengths
    l1 = 0.128
    l2 = 0.024
    l1c = 0.13023 # ua_link = combined l1 - l2 length
    l3 = 0.124    # fa_link
    l4 = 0.126    # tcp_link

    # base offsets
    x_offset = 0.012
    z_offset = 0.0595 + 0.017

    # joint offsets due to combined l1 - l2
    j1_offset = math.atan(l2/l1)
    j2_offset = math.pi/2.0 + j1_offset # includes +90 degrees offset, too

    # default return list
    angles = [0,0,0,0]

    # Calculate the shoulder pan angle from x and y coordinates
    j0 = math.atan(coords[1]/(coords[0] - x_offset))

    # Re-calculate target coordinated to the wrist joint (x', y', z')
    x = coords[0] - x_offset - l4 * math.cos(j0) * math.cos(gripper_angle)
    y = coords[1] - l4 * math.sin(j0) * math.cos(gripper_angle)
    z = coords[2] - z_offset + math.sin(gripper_angle) * l4

    # Solve the problem in 2D using x" and z'
    x = math.sqrt(y*y + x*x)

    # Let's calculate auxiliary lengths and angles
    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = math.pi - alpha
    # Apply law of cosines
    gamma = math.acos((l1c*l1c + c*c - l3*l3)/(2*c*l1c))

    j1 = math.pi/2.0 - alpha - gamma - j1_offset
    j2 = math.acos((l1c*l1c + l3*l3 - c*c)/(2*l1c*l3)) - j2_offset
    delta = math.pi - j2 - gamma - j2_offset

    j3 = math.pi + gripper_angle - beta - delta

    angles[0] = j0
    angles[1] = j1
    angles[2] = -j2
    angles[3] = j3

    return angles
```

Let's try it out, first start the simulation and move the red box closer, to 25cm distance on the X axis:
```bash
ros2 launch open_manipulator_mogi simulation_bringup.launch.py
```

![alt text][image15]

Then open the gripper:
```bash
ros2 run open_manipulator_mogi_py open_gripper
```
![alt text][image16]


And finally run the inverse kinematics node:
```bash
ros2 run open_manipulator_mogi_py inverse_kinematics
```
![alt text][image17]

Then we could grab and move the red box with the gripper.


# Unviersal Robots

```bash
https://github.com/MOGI-ROS/Universal_Robots_ROS2_GZ_Simulation
https://github.com/ros-industrial/ur_msgs humble
https://github.com/UniversalRobots/Universal_Robots_Client_Library
https://github.com/MOGI-ROS/Universal_Robots_ROS2_Description
https://github.com/MOGI-ROS/Universal_Robots_ROS2_Driver
```
## First try of the UR3e 

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

## UR3e with MoveIt

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```

## Adding a gripper 



## Updating MoveIt config

SRDF file add new home

## Gazebo world



## MoveIt commander


