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
    moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("ur_moveit_config") + "/config/moveit_cpp.yaml") 
    moveit_config_builder.robot_description_semantic(get_package_share_directory("ur_moveit_config") + "/srdf/ur_moveit_py.srdf")
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
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="home")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = ur_commander.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    ur_commander_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    ur_commander_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "shoulder_pan_joint": -1.0,
        "shoulder_lift_joint": 0.7,
        "elbow_joint": 0.7,
        "wrist_1_joint": -1.5,
        "wrist_2_joint": -0.7,
        "wrist_3_joint": 2.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=ur_commander.get_robot_model().get_joint_model_group("ur_manipulator"),
    )
    ur_commander_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    ur_commander_arm.set_goal_state(configuration_name="ready")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        ur_commander, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        ur_commander,
        ur_commander_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )


if __name__ == "__main__":
    main()