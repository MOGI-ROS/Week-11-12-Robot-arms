import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Create a publisher for the '/arm_controller/joint_trajectory' topic
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Create the JointTrajectory message
        self.trajectory_command = JointTrajectory()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.trajectory_command.joint_names = joint_names

        point = JointTrajectoryPoint()
        joint_angles = self.inverse_kinematics([0.25, 0, 0.075], 0)
        point.positions = joint_angles
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1

        self.trajectory_command.points = [point]

        # Publish the message
        self.get_logger().info('Publishing joint angles...')

    def send_joint_angles(self):

        while rclpy.ok():
            self.publisher.publish(self.trajectory_command)
            rclpy.spin_once(self, timeout_sec=0.1)

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


def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()

    try:
        node.send_joint_angles()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

