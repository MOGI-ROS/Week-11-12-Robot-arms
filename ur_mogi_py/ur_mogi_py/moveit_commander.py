#!/usr/bin/env python3
import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
# For collision objects
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import ObjectColor
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
# For configuration
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
    for i in range(10):
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
            time.sleep(sleep_time)
            break
        else:
            logger.error("Planning failed")

def add_collision_objects(planning_scene_monitor):
    """Helper function that adds collision objects to the planning scene."""
    object_positions = [
        (0.0, -0.6, 0.5),
        (0.3, 0.3, 0.7),
    ]
    object_dimensions = [
        (0.05, 0.3, 0.2),
        (0.05, 0.3, 0.2),
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        color = ObjectColor()
        color.id = "boxes"
        color.color = ColorRGBA()
        color.color.r = 1.0
        color.color.g = 0.0
        color.color.b = 0.0
        color.color.a = 0.5

        scene.apply_collision_object(collision_object, color)
        scene.current_state.update()  # Important to ensure the scene is updated

def add_collision_objects_2(planning_scene_monitor):
    """Helper function that adds collision objects to the planning scene."""
    object_positions = [
        (0.2, -0.3, 0.02),
    ]
    object_dimensions = [
        (0.03, 0.3, 0.04),
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        color = ObjectColor()
        color.id = "boxes"
        color.color = ColorRGBA()
        color.color.r = 0.0
        color.color.g = 0.0
        color.color.b = 1.0
        color.color.a = 0.5

        scene.apply_collision_object(collision_object, color)
        scene.current_state.update()  # Important to ensure the scene is updated

def add_grasp_objects(planning_scene_monitor):
    """Helper function that adds collision objects to the planning scene."""
    object_positions = [
        (0.32, -0.168, 0.05),
    ]
    object_dimensions = [
        (0.1, 0.03),
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "cylinder"

        for position, dimensions in zip(object_positions, object_dimensions):
            cylinder_pose = Pose()
            cylinder_pose.position.x = position[0]
            cylinder_pose.position.y = position[1]
            cylinder_pose.position.z = position[2]

            cylinder = SolidPrimitive()
            cylinder.type = SolidPrimitive.CYLINDER
            cylinder.dimensions = dimensions

            collision_object.primitives.append(cylinder)
            collision_object.primitive_poses.append(cylinder_pose)
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)

        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.link_name = "end_effector_link"
        attached_collision_object.object = collision_object
        attached_collision_object.touch_links = ["rh_p12_rn_l1", "rh_p12_rn_l2", "rh_p12_rn_r1", "rh_p12_rn_r2"]

        scene.process_attached_collision_object(attached_collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated

def detach_grasp_objects(planning_scene_monitor):
    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "cylinder"
        collision_object.operation = CollisionObject.REMOVE

        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.link_name = "end_effector_link"
        attached_collision_object.object = collision_object

        scene.process_attached_collision_object(attached_collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated

def main():

    ###################################################################
    # MoveIt Configuration
    ###################################################################
    moveit_config_builder = MoveItConfigsBuilder("ur")
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
    planning_scene_monitor = ur_commander.get_planning_scene_monitor()

    logger.info(30*"*")
    logger.info("Press Enter to start planning")
    logger.info(30*"*")
    input()

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    
    # set plan start state using predefined state
    #ur_commander_arm.set_start_state(configuration_name="up")
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="mogi_down")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 1: Down position reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Add collision objects to the planning scene
    ###########################################################################

    add_collision_objects(planning_scene_monitor)

    logger.info(30*"*")
    logger.info("Collision objects added")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 2 - set states with predefined string
    ###########################################################################
    
    # set plan start state using predefined state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="up")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 2: Init position reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 3 - set states with predefined string
    ###########################################################################
    
    # set plan start state using predefined state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="mogi_home")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 3: Home position reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 4 - close gripper with predefined string
    ###########################################################################
    
    # set plan start state using predefined state
    ur_commander_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_gripper.set_goal_state(configuration_name="closed")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 4: Gripper closed")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Remove collision objects from the planning scene
    ###########################################################################

    with planning_scene_monitor.read_write() as scene:
        scene.remove_all_collision_objects()
        scene.current_state.update()

    logger.info(30*"*")
    logger.info("Collision objects removed")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 5 - set joint angles
    ###########################################################################
    
    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    joint_values = {
        "shoulder_pan_joint": -1.5707,
        "shoulder_lift_joint": -1.5707,
        "elbow_joint": 0.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }
    robot_state.joint_positions = joint_values
    ur_commander_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 5: Joint angles reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 6 - set goal state with PoseStamped message
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
    logger.info("Plan 6: TCP pose reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 7 - open gripper with predefined string
    ###########################################################################

    # set plan start state using predefined state
    ur_commander_gripper.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_gripper.set_goal_state(configuration_name="open")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 7: Gripper open")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 8 - set goal state with cartesian constraints
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
    logger.info("Plan 8: Cartesian constraint reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Add grasp objects to the planning scene
    ###########################################################################

    add_grasp_objects(planning_scene_monitor)
    time.sleep(1.0)
    ur_commander_gripper.set_start_state_to_current_state()

    logger.info(30*"*")
    logger.info("Grasp objects added")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 9 - close gripper with joint angles
    ###########################################################################

    # set plan start state using predefined state
    ur_commander_gripper.set_start_state_to_current_state()

    joint_values = {
        "rh_r1_joint": 0.59,
    }
    robot_state.joint_positions = joint_values
    ur_commander_gripper.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_gripper, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 9: Gripper angle reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Add collision objects to the planning scene
    ###########################################################################

    add_collision_objects_2(planning_scene_monitor)

    logger.info(30*"*")
    logger.info("Collision objects added")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 10 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    ur_commander_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.924
    pose_goal.pose.orientation.y = -0.383
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = -0.4
    pose_goal.pose.position.z = 0.09

    ur_commander_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 10: Cartesian constraint reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Plan 11 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    ur_commander_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    ur_commander_arm.set_goal_state(configuration_name="up")

    # plan to goal
    plan_and_execute(ur_commander, ur_commander_arm, logger, sleep_time=3.0)

    logger.info(30*"*")
    logger.info("Plan 11: Home position reached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Detach graps object from the gripper
    ###########################################################################

    detach_grasp_objects(planning_scene_monitor)
    time.sleep(1.0)
    ur_commander_gripper.set_start_state_to_current_state()

    logger.info(30*"*")
    logger.info("Grasp object detached")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ###########################################################################
    # Remove collision objects from the planning scene
    ###########################################################################

    with planning_scene_monitor.read_write() as scene:
        scene.remove_all_collision_objects()
        scene.current_state.update()

    logger.info(30*"*")
    logger.info("Collision objects removed")
    logger.info(30*"*")

    input("Press Enter to continue...")

    ur_commander.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()