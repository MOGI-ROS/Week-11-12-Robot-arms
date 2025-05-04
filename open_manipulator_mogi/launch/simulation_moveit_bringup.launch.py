#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yoon, Sungho Woo

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def get_robot_model():
    robot_model = os.getenv("ROBOT_MODEL", "om_y")
    if robot_model not in ["om_x", "om_y", "om_y_follower"]:
        raise ValueError(f"Invalid ROBOT_MODEL: {robot_model}")
    return robot_model

def generate_launch_description():
    # Launch Arguments
    robot_model = get_robot_model()

    if robot_model == "om_x":
        urdf_file = "open_manipulator_x"
        urdf_folder = "om_x"
    else:
        urdf_file = "open_manipulator_y"
        urdf_folder = "om_y"

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_open_manipulator_description = os.path.join(
        get_package_share_directory('open_manipulator_description'))

    pkg_open_manipulator_bringup = os.path.join(
        get_package_share_directory('open_manipulator_bringup'))
    
    pkg_open_manipulator_mogi = get_package_share_directory('open_manipulator_mogi')

    pkg_open_manipulator_moveit_config = get_package_share_directory('open_manipulator_moveit_config')

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ["GZ_SIM_RESOURCE_PATH"], ':',
            os.path.join(pkg_open_manipulator_bringup, 'worlds'), ':' +
            str(Path(pkg_open_manipulator_description).parent.resolve())
            ]
        )

    gz_bridge_params_path = os.path.join(
        pkg_open_manipulator_mogi,
        'config',
        'gz_bridge.yaml'
    )

    xacro_file = os.path.join(pkg_open_manipulator_description,
                              "urdf",
                              urdf_folder,
                              f"{urdf_file}.urdf.xacro")

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    world_arg = LaunchDescription([
                DeclareLaunchArgument('world', default_value='world.sdf',
                          description='Name of the Gazebo world file to load'),
           ]
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )

    z_arg = DeclareLaunchArgument(
        'z', default_value='1.02',
        description='z coordinate of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_open_manipulator_mogi,
            'worlds',
            LaunchConfiguration('world')
        ]),
        TextSubstitution(text=' -r -v -v1')],
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'om',
                   '-allow_renaming', 'true'
                   '-use_sim','true'],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_urdf_node,
                on_exit=[load_joint_state_controller],
            )
    )
    
    robot_controllers_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_arm_controller,
                        load_gripper_controller],
            )
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

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
    moveit_event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[move_group_launch,
                        rviz_launch],
            )
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(gazebo_resource_path)
    launchDescriptionObject.add_action(joint_state_broadcaster_event_handler)
    launchDescriptionObject.add_action(robot_controllers_event_handler)
    launchDescriptionObject.add_action(gazebo_launch)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(moveit_event_handler)

    return launchDescriptionObject

