from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import fabs, cos, sqrt
    tau = pi/2.0
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

PI = np.pi - 0.01 # for more readability, but to prevent joints from getting to close to their max values 

far_left_theta1 = PI/6
r_max_theta2 = -PI/2.8
r_max_theta3 = PI/3.8


beginning_frame =  (   PI/2,   -PI/2,   2*PI/4, 0, 0, 0) #before reaching forward
lean_forward = (PI/2, -PI/3, PI/3-PI/12, 0,0,0)
pick_up = (PI/2, -PI/3, PI/3+PI/12, 0,0,0)
orient_above_board = (0, -PI/2.5, 0, 0,0,0)
orient_far_left_above = (far_left_theta1, -PI/2.5, 0, 0,0,0)
orient_far_left_extend = (far_left_theta1, -PI/3, 0,0,0,0 )


class MoveGroupPlayGame(object):

    def __init__(self):
        super(MoveGroupPlayGame, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_play_game", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"

        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

 
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()


        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, joint_vals): 
        # joint_vals is an array that contains the 
        # desired angle for each joint in the desired pose, in format:
        # (theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal = joint_vals

        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return 

    
    # takes in a set of poses, and the starting/ending orientation 
    # for the letter we want to draw 
    def draw(self, joint_orientations, idle_frame):
        print("   orienting...")
        for idx, joint_vals in enumerate(joint_orientations):
            print("going to state ",idx)
            self.go_to_joint_state(joint_vals)
        return 


# does not use CV; takes user's input move to update the board state
def main():
    currState = "4"
    finished = False
    game = MoveGroupPlayGame() #create an instance of a Drawing 

    try:
        # while loop continues until either the user or robot wins
        while finished is False:
            print("The robot is red; you are yellow!")
            column = input(
                "Make your move by entering the column where you want to drop the chip!\n"
            )
            currState += column
            print(currState)

            # input(
            #     "Press `Enter` to pick up chip"
            # )  
            # game.go_to_joint_state(beginning_frame)
            # game.go_to_joint_state(lean_forward)
            # game.go_to_joint_state(pick_up)
            # game.go_to_joint_state(lean_forward)

            # input(
            #     "Press `Enter` to align above center of board"
            # )  

            # game.go_to_joint_state(orient_above_board)

            # input(
            #     "Press `Enter` to align above far left column of board"
            # )  

            # game.go_to_joint_state(orient_far_left_above)
            # game.go_to_joint_state(orient_far_left_extend)


            print("Hooray! ;\)")


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()