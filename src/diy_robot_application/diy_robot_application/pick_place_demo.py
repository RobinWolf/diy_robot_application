import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #6D Transformation 
import numpy as np
import time


        
def main(args=None):
    # initialize ros communications for a given context 
    rclpy.init(args=args)

    # initialize robot client node --> this will create clients in the RobotConnection class which call services to communicate with moveit
    robot = RobotClient(is_simulation=True)     # if not connected to the real robot set is_simulation=True 
    
    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # move robot to home position
    robot.home()

    # open the gripper
    robot.toggle_gripper(True)

    # move over the pick position (absloute world movement)
    robot.ptp(Affine((0.0, -0.2, 0.05),
                     (-np.pi, 0, 0)))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_1',current_pose)
    
    # move down to the pick position
    movement_tcp = Affine((0.0, 0.0, 0.13))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)
    print('pos_2 (pick)',target_pose)

    # wait 1s
    time.sleep(1)

    # close the gripper
    robot.toggle_gripper(False)

    # move up from the pick position (tcp movement)
    movement_tcp = Affine((0.0, 0.0, -0.13))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)
    print('pos_3',target_pose)

    # move over the place position (absolute world movement)
    robot.ptp(Affine((0.1, -0.2, 0.04), 
                      (-np.pi, 0, 0)))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print('pos_4',current_pose)
    
    # move down to the place position (tcp movement)
    movement_tcp = Affine((0.0, 0.0, 0.13))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)
    print('pos_5 (place)',target_pose)

    # open the gripper
    robot.toggle_gripper(False)

    # wait 1s
    time.sleep(1)

    # move up from the place position (tcp movement)
    movement_tcp = Affine((0.0, 0.0, -0.13))
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)  
    print('pos_6',target_pose)

    
    # home the robot, move back to the home position and diasble the joint drive
    robot.home()

    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()










