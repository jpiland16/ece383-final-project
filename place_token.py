#!/usr/bin/python3
from object_control import ObjectController, TokenColor
from attacher import AttachDetachHelper
from demo_ee import MovableRobot, all_close
import time
from typing import List, Callable
import rospy
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from std_srvs.srv import Empty

from optional_print import optional_print as print

from geometry_msgs.msg import Pose, Quaternion, Point
import tf.transformations as tr
import numpy as np

def robot_has_moved(old_joint_values: List[float], now_joint_values: List[float], tolerance: float = 0.25):
    for a, b in zip(old_joint_values, now_joint_values):
        if abs(a - b) > tolerance:
            return True
    return False

def move_with_verification(move_fn: Callable[[], None], motion_check_function: Callable[[], bool], final_check_function: Callable[[], bool]):
    CONUTER_FREQUENCY = 20 # Hz
    MOTION_CHECK_FREQUENCY = 1 # Hz
    
    move_fn()

    counter = 1
    reissue_count = 0
    while True:
        if final_check_function():
            break

        # else, not close
        if counter % (CONUTER_FREQUENCY * MOTION_CHECK_FREQUENCY) == 0: # and not motion_check_function():
            reissue_count += 1
            print(f"Robot appears to have ignored command. Re-issuing... ({reissue_count})")
            move_fn()

        print(f"Waiting for robot to achieve goal ({counter/20:.2f} sec elapsed)\r", end="", flush=True)
        time.sleep(1 / CONUTER_FREQUENCY)
        counter += 1

    print("Move complete!                                                     ")

def move_robot_with_verification(move_group, move_fn: Callable[[], None], final_check_function: Callable[[], bool]):
    # robot as opposed to end-effector    
    starting_joint_values = move_group.get_current_joint_values()
    def check_motion():
        nonlocal starting_joint_values
        current_joint_values = move_group.get_current_joint_values()
        has_moved = robot_has_moved(starting_joint_values, current_joint_values)
        # if has_moved:
        #     starting_joint_values = move_group.get_current_joint_values()
        return has_moved
    move_with_verification(move_fn, check_motion, final_check_function)

def stabilize_initial_state():

    rospy.ServiceProxy("/gazebo/pause_physics", Empty)()

    model_config_service = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

    req = SetModelConfigurationRequest(
        "robot",
        "ur5e_robot",
        ["elbow_joint", "finger_joint", "shoulder_pan_joint", "shoulder_lift_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        [-0.7943, 0.0226, 0.4974, 0.0035, 0.0883, 0.0318, -0.0566]
    )

    res = model_config_service(req)
    print(res)

    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()

def invert_quaternion(q: Quaternion):
    return Quaternion(*tr.quaternion_inverse([q.x, q.y, q.z, q.w]))

def multiply_quaternions(q1: Quaternion, q2: Quaternion):
    return Quaternion(*tr.quaternion_multiply(
        [q1.x, q1.y, q1.z, q1.w],
        [q2.x, q2.y, q2.z, q2.w],
    ))

def robot_play_token_in_column(robot: MovableRobot, column: int, expected_height: int, token_color: TokenColor,
                               drop_token_callback: Callable[[], bool], lodge_token_callback: Callable[[], bool]):
    """
    Notes: 

        - `drop_token_callback` and `lodge_token_callback` are called when the
           token is either dropped (appears below the board) or lodged in the
           board (not in expected range for z), respectively. Both functions
           should return `True` if the placement should be tried again, and
           `False` otherwise. The timing of this attempt can, of course, be
           controlled by delaying the return of the callback function.
    """

    if (not (1 <= column <= 7)) or (int(column) != column):
        raise ValueError(f"Invalid column id: {column}")
    if (not (1 <= expected_height <= 6)) or (int(expected_height) != expected_height):
        raise ValueError(f"Invalid expected height: {expected_height}")

    attach_detach_helper = AttachDetachHelper()
    object_controller = ObjectController()

    move_group = robot.instance.move_group
    ee = robot.instance.ee_group
    ee.go([0])

    initial_target_joint_values = [3.376873033991288, -1.7, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]

    def initial_command():
        move_group.go(initial_target_joint_values)
    def initial_command_check():
        now_joint_values = move_group.get_current_joint_values()
        return all_close(initial_target_joint_values, now_joint_values, 0.03)
    move_robot_with_verification(move_group, initial_command, initial_command_check)
    
    print("Robot is ready!")


    # object_controller.delete_all_free_models()
    token_name, res = object_controller.spawn_token_at_location(token_color, 0.616, 0.007, 0.08)

    print(f"Spawned token {token_name}!")

    robot.go(x = 0.62, y = 0.007, z = 0.5, qx = 0, qy = -1, qz = 0, qw = 0)

    robot.go(z = 0.232)

    ee.go([0.62])

    attach_detach_helper.attach_token(token_name)
    print("Token attached!")

    Z_TARGET = 0.53
    def lift_token():
        robot.go(z = Z_TARGET)
    def lift_token_check():
        current_pose: Pose = move_group.get_current_pose().pose
        return abs(current_pose.position.z - Z_TARGET) < 0.02
    move_robot_with_verification(move_group, lift_token, lift_token_check)

    
    # position check
    def align_token(x_target, y_target):
        
        def token_is_in_place():
            pose = object_controller.get_model_state(token_name).pose
            position = pose.position
            orientation = pose.orientation
            return (
                abs(position.x - x_target) < 0.0006 and
                abs(position.y - y_target) < 0.0006 and
                # abs(position.z - z_target) < 0.008  and
                abs(orientation.x - np.sqrt(2)/2) < 0.006 and
                abs(orientation.y -            0) < 0.006 and
                abs(orientation.z -            0) < 0.006 and
                abs(orientation.w - np.sqrt(2)/2) < 0.006
            )
        
        counter = 0
        
        while not token_is_in_place() and counter < 30: # Spend at most one minute doing this
            counter += 1
            
            print(f"Token is not aligned to target. Moving... ({counter})")

            if False:
                token_pose = object_controller.get_model_state(token_name).pose
                print("Pose of token was: ")
                print(token_pose)
                input("Press ENTER to continue...")
            
            token_orientation = object_controller.get_model_state(token_name).pose.orientation
            ee_orientation = object_controller.get_robot_ee_pose().orientation


            transform_ee_to_token = multiply_quaternions(
                invert_quaternion(ee_orientation),
                token_orientation
            )

            target_token_orientation = Quaternion(np.sqrt(2)/2, 0, 0, np.sqrt(2)/2)

            robot_target_tilt = multiply_quaternions(
                target_token_orientation,
                invert_quaternion(transform_ee_to_token)
            )

            robot.go(
                qx = robot_target_tilt.x, 
                qy = robot_target_tilt.y, 
                qz = robot_target_tilt.z, 
                qw = robot_target_tilt.w
            )

            token_position = object_controller.get_model_state(token_name).pose.position
            robot_position = object_controller.get_robot_ee_pose().position

            robot.go(
                x = x_target + (robot_position.x - token_position.x),
                y = y_target + (robot_position.y - token_position.y),
                # z = z_target + (robot_position.z - token_position.z)
            )

            time.sleep(2)

    X_TARGET =  0.204  + 0.0348 * (column - 1)
    Y_TARGET = -0.2976

    def move_above_board():
        current_pose = move_group.get_current_pose().pose
        pose_above_board = Pose(
            Point(X_TARGET, Y_TARGET, Z_TARGET),
            Quaternion(0, -1, 0, 0)
        )
        (plan, fraction) = move_group.compute_cartesian_path(
            [current_pose, pose_above_board], 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan)

    move_above_board()

    robot.go(X_TARGET, Y_TARGET, Z_TARGET) # needed to update parameters in this class, the robot should already be at this position

    align_token(X_TARGET, Y_TARGET)
    token_position_z = object_controller.get_model_state(token_name).pose.position.z
    # Don't hit the board with the token if it is hanging lower than usual!
    # My guess is that this occurs due to the physics slowing down with more tokens, then the token tilting less in its holder... idk...
    token_droop = 0.3808 - token_position_z
    robot.go(z = 0.52 + token_droop)
    align_token(X_TARGET, Y_TARGET)

    print("Ready to drop token! Its current pose is")
    print(object_controller.get_model_state(token_name).pose)

    attach_detach_helper.detach_token(token_name)
    time.sleep(2)
    ee.go([0.5])
    robot.go(z = 0.6)

    wait_seconds = 20   # if token appears lodged

    while object_controller.get_model_state(token_name).pose.position.z > 0.349:
        print(f"Token is still above the board. Waiting {wait_seconds} second(s) for it to fall...", end="", flush=True)
        time.sleep(1)
        wait_seconds -= 1
        if wait_seconds == 0:
            break 

    z_pos = object_controller.get_model_state(token_name).pose.position.z

    if z_pos < 0.157:
        # below board
        print("TOKEN DROPPED! Waiting for callback decision to retry...")
        if drop_token_callback():
            # if callback comes back True, then try again
            attach_detach_helper.link_board_to_token(token_name)
            robot_play_token_in_column(robot, column, expected_height, token_color, drop_token_callback, lodge_token_callback)
        else:
            attach_detach_helper.link_board_to_token(token_name)
    else:
        # check for expected height
        expected = 0.173 + 0.032 * (expected_height - 1)
        min_expected = expected - 0.008   # 8 mm = 1/2 of radius of token
        max_expected = expected + 0.008
        if not (min_expected < z_pos < max_expected):
            # lodged token
            print("TOKEN LODGED IN BOARD! Waiting for callback decision to retry...")
            if lodge_token_callback():
                # if callback comes back True, then try again
                robot_play_token_in_column(robot, column, expected_height, token_color, drop_token_callback, lodge_token_callback)
            attach_detach_helper.link_board_to_token(token_name)
        else:
            print("Token placement SUCCESS.")
            attach_detach_helper.link_board_to_token(token_name)

    print("Token affixed to board. Physics should be relatively fast still.")


