import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #6D Transformation 
import numpy as np
import time


        
def main(args=None):
    # initialize ros communications for a given context 
    rclpy.init(args=args)

    # initialize robot client node --> this will create clients in the RobotConnection class which call services to communicate with moveit
    robot = RobotClient(is_simulation=False)     # if not connected to the real robot set is_simulation=True 
    
    # define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # move robot to home position
    robot.home()

    # open the gripper
    robot.toggle_gripper(False)

    # move above the pick position (absloute world movement)
    robot.ptp_joint([0.0, 0.157, 2.31, -0.9, -1.57, 0.0])

    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_1',current_pose)

    # move down to the pick position (joint 2 movement)
    robot.ptp_joint([0.0, 0.557, 2.31, -1.2, -1.57, 0.0])
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_2 (pick)',current_pose)

    # close the gripper
    robot.toggle_gripper(True)

    # wait
    time.sleep(2)

    # move up
    robot.ptp_joint([0.0, 0.157, 2.31, -0.9, -1.57, 0.0])
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_3',current_pose)

    # move the first axis by 45 deg to place pos
    robot.ptp_joint([np.pi/4, 0.157, 2.31, -0.9, -1.57, 0.0])
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_4',current_pose)

    # move down
    robot.ptp_joint([np.pi/4, 0.557, 2.31, -1.2, -1.57, 0.0])
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_5 (place)',current_pose)

    # open the gripper
    robot.toggle_gripper(False)

    # wait
    time.sleep(2)

    # move up
    robot.ptp_joint([np.pi/4, 0.157, 2.31, -0.9, -1.57, 0.0])
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_6',current_pose)

    time.sleep(5)

    # home the robot, move back to the home position and diasble the joint drive
    robot.home()
    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


    ## NOTE: lin movement is currently not working with the real hardware, we are still looking for the bug (01.03.24)










