import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np


def complex_movement_example(robot, pose):
    offset = Affine((0, 0, 0.1))
    new_pose = offset * pose
    robot.lin(new_pose)

    robot.ptp(pose)


def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)

    # initialize robot client node
    # if not connected to the real robot set is_simulation=True
    robot = RobotClient(is_simulation=True)
    # home joint positions of the robot in Jointroom (Winkel in Rad default)
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)


    # destroy the robot node
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()
