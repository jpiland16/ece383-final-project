from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import requests
import cv2

# try:
from math import pi, tau, dist, fabs, cos
# except:  # For Python 2 compatibility
#     from math import fabs, cos, sqrt
#     tau = pi/2.0
#     def dist(p, q):
#         return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


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


# _________________________________________________________________________________________
# CV Code

# Display image (remember, this only works in FastX!).
def imgshow(name,img):
    cv2.imshow(name,img)
    cv2.moveWindow(name,200,200)
    cv2.waitKey(0)

def get_curr_board_state_using_cv():
    # Read and process image.
    input_img = cv2.imread("ex1.JPG")

    # Resize input image.
    new_width = 500
    img_h,img_w,_ = input_img.shape
    scale = new_width / img_w
    img_w = int(img_w * scale)
    img_h = int(img_h * scale)
    # See https://medium.com/@wenrudong/what-is-opencvs-inter-area-actually-doing-282a626a09b3 for what cv2.INTER_AREA
    # interpolation method means.
    input_img = cv2.resize(input_img, (img_w, img_h), interpolation=cv2.INTER_AREA)
    original_img = input_img.copy()
    imgshow("Original Image (Resized)", original_img)

    # Apply bilateral filter.
    bilaterally_filtered_img = cv2.bilateralFilter(input_img, 15, 190, 190) 
    imgshow("image after a bilateral filter was applied", bilaterally_filtered_img)

    # Outline the edges.
    edged_img = cv2.Canny(bilaterally_filtered_img, 75, 150) 
    imgshow("image after applying edge detection", edged_img)

    # Find the spots.
    contours, hierarchy = cv2.findContours(edged_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # edges to contours

    contour_list = []
    rect_list = []
    position_list = []

    for contour in contours:
        approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True) # Contour Polygons
        area = cv2.contourArea(contour)
        
        rect = cv2.boundingRect(contour) # Polygon bounding rectangles
        x_rect,y_rect,w_rect,h_rect = rect
        x_rect +=  w_rect/2
        y_rect += h_rect/2
        area_rect = w_rect*h_rect
        
        if ((len(approx) > 8) & (len(approx) < 23) & (area > 250) & (area_rect < (img_w*img_h)/5)) & (w_rect in range(h_rect-10,h_rect+10)): # Circle conditions
            contour_list.append(contour)
            position_list.append((x_rect,y_rect))
            rect_list.append(rect)

    img_circle_contours = original_img.copy()
    cv2.drawContours(img_circle_contours, contour_list,  -1, (0,255,0), thickness=1) # Display Circles
    for rect in rect_list:
        x,y,w,h = rect
        cv2.rectangle(img_circle_contours,(x,y),(x+w,y+h),(0,0,255),1)

    imgshow('Circles Detected',img_circle_contours)

    # Interpolate Grid
    rows, cols = (6,7)
    mean_w = sum([rect[2] for r in rect_list]) / len(rect_list)
    mean_h = sum([rect[3] for r in rect_list]) / len(rect_list)
    position_list.sort(key = lambda x:x[0])
    max_x = int(position_list[-1][0])
    min_x = int(position_list[0][0])
    position_list.sort(key = lambda x:x[1])
    max_y = int(position_list[-1][1])
    min_y = int(position_list[0][1])
    grid_width = max_x - min_x
    grid_height = max_y - min_y
    col_spacing = int(grid_width / (cols-1))
    row_spacing = int(grid_height / (rows - 1))

    # Find Colour Masks
    img_hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV) # Convert to HSV space

    lower_red = np.array([150, 150, 100])  # Lower range for red colour space
    upper_red = np.array([255, 255, 255])  # Upper range for red colour space
    mask_red = cv2.inRange(img_hsv, lower_red, upper_red)
    img_red = cv2.bitwise_and(input_img, input_img, mask=mask_red)
    imgshow("Red Mask",img_red)

    lower_yellow = np.array([10, 150, 100])
    upper_yellow = np.array([60, 255, 255])
    mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    img_yellow = cv2.bitwise_and(input_img, input_img, mask=mask_yellow)
    imgshow("Yellow Mask",img_yellow)

    # Identify Colours
    grid = np.zeros((rows,cols))
    id_red = 1
    id_yellow = -1
    img_grid_overlay = original_img.copy()
    img_grid = np.zeros([img_h,img_w,3], dtype=np.uint8)

    for x_i in range(0,cols):
        x = int(min_x + x_i * col_spacing)
        for y_i in range(0,rows):
            y = int(min_y + y_i * row_spacing)
            r = int((mean_h + mean_w)/5)
            img_grid_circle = np.zeros((img_h, img_w))
            cv2.circle(img_grid_circle, (x,y), r, (255,255,255),thickness=-1)
            img_res_red = cv2.bitwise_and(img_grid_circle, img_grid_circle, mask=mask_red)
            img_grid_circle = np.zeros((img_h, img_w))
            cv2.circle(img_grid_circle, (x,y), r, (255,255,255),thickness=-1)
            img_res_yellow = cv2.bitwise_and(img_grid_circle, img_grid_circle,mask=mask_yellow)
            cv2.circle(img_grid_overlay, (x,y), r, (0,255,0),thickness=1)
            if img_res_red.any() != 0:
                grid[y_i][x_i] = id_red
                cv2.circle(img_grid, (x,y), r, (0,0,255),thickness=-1)
            elif img_res_yellow.any() != 0 :
                grid[y_i][x_i] = id_yellow
                cv2.circle(img_grid, (x,y), r, (0,255,255),thickness=-1)
        
    print('Grid Detected:\n', grid)
    imgshow('Img Grid Overlay',img_grid_overlay)
    imgshow('Img Grid',img_grid)





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

            solver = "https://connect4.gamesolver.org/solve"
            params = {"pos": currState,}
            result = requests.get(solver, params=params).text

            get_curr_board_state_using_cv()

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