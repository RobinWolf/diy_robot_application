import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np

#########################################################################################################################################
##                                                      test scedules                                                                  ##
#########################################################################################################################################

def joint_absolute_ptp_test(robot):
    #home the robot before you start the test scedule
    robot.home()

    #robot joint movement /axis 2  pi/4 = 45° , floats required!
    robot.ptp_joint([np.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0])

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #robot joint movement /axis 1  pi/2 = 90°
    robot.ptp_joint([np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0])

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    robot.home()


def cartesian_absolute_ptp_test(robot):
    #home the robot before you start the test scedule
    robot.home()

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.ptp(Affine((0.03607227, 0.15249834, 1.20871037), #TODO figure out nice positions in rviz
                      (0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #use quarternions instead of euler angles for the orientation
    robot.ptp(Affine((0.03607227, 0.15249834, 1.20871037), #TODO figure out nice positions in rviz --> schlau wählen famit lin/ptp unterschied sichtbar
                      (0, 0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    robot.home()


def cartesian_absolute_lin_test(robot):
    #home the robot before you start the test scedule
    robot.home()

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.lin(Affine((0.03607227, 0.15249834, 1.20871037), #TODO figure out nice positions in rviz
                      (0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #use quarternions instead of euler angles
    robot.lin(Affine((0.03607227, 0.15249834, 1.20871037), #TODO figure out nice positions in rviz
                      (0, 0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    robot.home()


    ### did you notice the difference in choosen trajectories for the same target posistions between ptp and lin ?

def cartestan_relative_world_test(robot):
    robot.home()

    # define the relative movement of the tcp in world-coordinates: translation in world coordinate system (0.1 in z_world direction)
    movement_world = Affine((0, 0, 0.1))

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    # apply translation to current pose given in world coordinates (look at the multiplication order -> this has to be in order to matrix-operations!)
    target_pose = movement_world * current_pose
    robot.lin(target_pose)

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    robot.home()


def cartestan_relative_tcp_test(robot):
    robot.home()

    # define the relative movement of the tcp in tcp-coordinates: translation in tcp coordinate system (0.1 in z_world direction)
    movement_tcp = Affine((0, 0, 0.1))

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    # apply translation to current pose given in world tcp (look at the multiplication order -> this has to be in order to matrix-operations!)
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    robot.home()


def gripper_test(robot):
    #home the robot before you start the test scedule
    robot.home()

    #open the gripper
    robot.toggle_gripper(False)

    #close the gripper
    robot.toggle_gripper(True)



#########################################################################################################################################
##                                                           main                                                                      ##
#########################################################################################################################################
        
def main(args=None):
    # initialize ros communications for a given context 
    rclpy.init(args=args)

    # initialize robot client node --> this will create clients in the RobotConnection class which call services to communicate with moveit
    robot = RobotClient(is_simulation=True)     # if not connected to the real robot set is_simulation=True 
    
    # move robot to home position (use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.home()

    # run one of the test scedules to make yourself familiar with the different movements
    joint_absolute_ptp_test(robot)

    robot.home()


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()










