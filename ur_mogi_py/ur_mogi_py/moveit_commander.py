#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.utils import create_params_file_from_dict
from ament_index_python import get_package_share_directory

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():


    moveit_config_builder = MoveItConfigsBuilder("ur")
    # This line is key!!!
    moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("ur_mogi") + "/config/moveit_cpp.yaml") 
    moveit_config_builder.robot_description_semantic(get_package_share_directory("ur_mogi") + "/config/moveit_cpp.srdf")
    moveit_config_builder.trajectory_execution(get_package_share_directory("ur_moveit_config") + "/config/moveit_controllers.yaml")
    moveit_config_builder.planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
    moveit_config_dict.update({'use_sim_time' : True})

    file = create_params_file_from_dict(moveit_config_dict, "/**")

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur_commander = MoveItPy(node_name="moveit_py", launch_params_filepaths=[file])
    ur_commander_arm = ur_commander.get_planning_component("ur_manipulator")
    ur_commander_gripper = ur_commander.get_planning_component("gripper")
    logger.info("MoveItPy instance created")

    robot_model = ur_commander.get_robot_model()
    robot_state = RobotState(robot_model)

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="mogi_home")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 1: Home position reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 2 - close gripper with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_gripper.set_goal_state(configuration_name="closed")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 2: Gripper closed")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 3 - set joint angles
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set constraints message
    #from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "shoulder_pan_joint": -1.5707,
        "shoulder_lift_joint": -1.5707,
        "elbow_joint": 0.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }
    robot_state.joint_positions = joint_values
    #joint_constraint = construct_joint_constraint(
    #    robot_state=robot_state,
    #    joint_model_group=ur_commander.get_robot_model().get_joint_model_group("ur_manipulator"),
    #)
    #ur_commander_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
    ur_commander_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 3: Joint angles reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 4 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.924
    pose_goal.pose.orientation.y = -0.383
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0
    pose_goal.pose.position.x = 0.32
    pose_goal.pose.position.y = -0.17
    pose_goal.pose.position.z = 0.3
    ur_commander_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 4: TCP coordinates reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 5 - open gripper with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_gripper.set_goal_state(configuration_name="open")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 5: Gripper open")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 6 - set goal state with cartesian constraints
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_link_constraint

    cartesian_constraint = construct_link_constraint(
        link_name="end_effector_link",
        source_frame="base_link",
        cartesian_position=[0.32, -0.17, 0.08],
        cartesian_position_tolerance=0.002,
        orientation=[0.924, -0.383, 0.0, 0.0],
        orientation_tolerance=0.001,
    )
    ur_commander_arm.set_goal_state(motion_plan_constraints=[cartesian_constraint])

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 6: Cartesian movement reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 7 - close gripper with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_gripper.set_start_state_to_current_state()

    joint_values = {
        "rh_r1_joint": 0.59,
    }
    robot_state.joint_positions = joint_values
    ur_commander_gripper.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 7: Gripper joint angle reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

if __name__ == "__main__":
    main()