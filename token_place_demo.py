from object_control import ObjectController, TokenColor
from attacher import AttachDetachHelper
from demo_ee import MovableRobot, all_close
import time
from typing import List, Callable

from geometry_msgs.msg import Pose

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
        if counter % (CONUTER_FREQUENCY * MOTION_CHECK_FREQUENCY) == 0 and not motion_check_function():
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
        current_joint_values = move_group.get_current_joint_values()
        return robot_has_moved(starting_joint_values, current_joint_values)
    move_with_verification(move_fn, check_motion, final_check_function)

def main():
    attach_detach_helper = AttachDetachHelper()
    object_controller = ObjectController()

    object_controller.delete_all_free_models()
    token_name, res = object_controller.spawn_token_at_location(TokenColor.RED, 0.6, 0.007, 0.1)

    robot = MovableRobot()
    move_group = robot.instance.move_group
    ee = robot.instance.ee_group
    ee.go([0])

    # initial_target_joint_values = [3.376873033991288, -2.0155372301770793, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]
    initial_target_joint_values = [3.376873033991288, -1.7, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]

    def initial_command():
        move_group.go(initial_target_joint_values)
    def initial_command_check():
        now_joint_values = move_group.get_current_joint_values()
        return all_close(initial_target_joint_values, now_joint_values, 0.03)
    move_robot_with_verification(move_group, initial_command, initial_command_check)
    
    print("Robot is ready!")

    robot.go(x = 0.62, y = 0.007, z = 0.5, qx = 0, qy = -1, qz = 0, qw = 0)

    robot.go(z = 0.232)

    ee.go([0.62])

    attach_detach_helper.attach_token(token_name)
    print("Token attached!")

    Z_TARGET = 0.5
    def lift_token():
        robot.go(z = Z_TARGET)
    def lift_token_check():
        current_pose: Pose = move_group.get_current_pose().pose
        return abs(current_pose.position.z - Z_TARGET) < 0.02
    move_robot_with_verification(move_group, lift_token, lift_token_check)

    robot.go(x = 0.29, y = -0.3, z = 0.55)

    attach_detach_helper.detach_token(token_name)
    ee.go([0.5])

if __name__ == "__main__":
    main()

