import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np
import time

#########################################################################################################################################
##                                                      test scedules                                                                  ##
#########################################################################################################################################

def joint_absolute_ptp_test(robot):

    #robot joint movement /axis 2  pi/4 = 45° , floats required!
    robot.ptp_joint([0.0, np.pi/4, 0.0, 0.0, 0.0, 0.0])

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #robot joint movement /axis 1  pi/2 = 90°
    robot.ptp_joint([np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0])

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)




def cartesian_absolute_ptp_test(robot):

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.ptp(Affine((-0.3, -0.15, 0.2), 
                      (-np.pi, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #use quarternions instead of euler angles for the orientation
    robot.ptp(Affine((0.3, -0.15, 0.2), 
                      (1, 0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)




def cartesian_absolute_lin_test(robot):

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.ptp(Affine((-0.3, -0.15, 0.2), 
                      (-np.pi, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    #use quarternions instead of euler angles, move in a line to the next pose
    robot.lin(Affine((0.3, -0.15, 0.2), 
                      (1, 0, 0, 0)))
    
    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    ### did you notice the difference in choosen trajectories for the same target posistions between ptp and lin ?

def cartestan_relative_world_test(robot):

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.ptp(Affine((-0.2, -0.2, 0.3), 
                      (-np.pi, 0, 0)))

    # define the relative movement of the tcp in world-coordinates: translation in world coordinate system (0.2 in z_world direction)
    movement_world = Affine((0, 0, 0.2))

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    # apply translation to current pose given in world coordinates (look at the multiplication order -> this has to be in order to matrix-operations!)
    target_pose = movement_world * current_pose
    robot.lin(target_pose)

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)




def cartestan_relative_tcp_test(robot):

    # robot ptp movement to given cartesian pose                                                
    # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w) or euler-angles (r, p, y)
    robot.ptp(Affine((-0.2, -0.2, 0.3), 
                      (-np.pi, 0, 0)))

    # define the relative movement of the tcp in tcp-coordinates: translation in tcp coordinate system (0.1 in z_world direction)
    movement_tcp = Affine((0, 0, 0.2))

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

    # apply translation to current pose given in world tcp (look at the multiplication order -> this has to be in order to matrix-operations!)
    target_pose = current_pose * movement_tcp
    robot.lin(target_pose)

    #print current transform/ pose of the tcp_link
    current_pose = robot.node.get_transform('grip_tcp_link', 'world')
    print(current_pose)

## did you noticei the difference in relative movement according to the tcp or the world?



def gripper_test(robot):

    #open the gripper
    robot.toggle_gripper(False)

    #wait
    time.sleep(3)

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
    
    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # move robot to home position
    robot.home()

    # run one of the test scedules to make yourself familiar with the different movements
    # please notice: motion-planning is a probabilistic procedure and taken trayectories to do the movements are not the same everytime you run the test scedules
    joint_absolute_ptp_test(robot)

    time.sleep(5)

    robot.home()

    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()










