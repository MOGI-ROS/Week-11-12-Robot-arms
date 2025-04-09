import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperCommandActionClient(Node):
    def __init__(self):
        super().__init__('gripper_command_action_client')

        # Action client for GripperCommand
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        self.gripper_position = 0.0
        self.gripper_max = 0.019
        self.gripper_min = -0.01

    def send_gripper_command(self):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self.gripper_position
        goal_msg.command.max_effort = 10.0

        self.get_logger().info(f'Sending gripper command: {goal_msg.command.position}')
        self.gripper_client.wait_for_server()
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)


def main(args=None):
    rclpy.init(args=args)
    node = GripperCommandActionClient()
    node.gripper_position = -0.005 # Close gripper to grab objects on table

    try:
        node.send_gripper_command()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()