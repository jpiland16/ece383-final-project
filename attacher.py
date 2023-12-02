import rospy
from gazebo_ros_link_attacher.srv import Attach
from object_control import ObjectController
from typing import Callable
import time
from optional_print import optional_print as print

class AttachDetachHelper():
    def __init__(self):
        self._attach_service = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self._detach_service = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    def attach_token(self, token_name: str):
        return self._attach_service(
            "robot",
            "wrist_3_link",
            token_name,
            "token_link"
        )

    def detach_token(self, token_name: str):
        return self._detach_service(
            "robot",
            "wrist_3_link",
            token_name,
            "token_link"
        )

    def link_board_to_token(self, token_name: str):
        return self._attach_service(
            "robot",
            "board_link",
            token_name,
            "token_link"
        )

    def wait_for_token_to_fall(self, token_name: str, expected_height: int, object_controller: ObjectController, drop_token_callback: Callable[[], bool], lodge_token_callback: Callable[[], bool], redo_function: Callable):

        if (not (1 <= expected_height <= 6)) or (int(expected_height) != expected_height):
            raise ValueError(f"Invalid expected height: {expected_height}")

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
                self.link_board_to_token(token_name)
                redo_function()
            else:
                self.link_board_to_token(token_name)
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
                    redo_function()
                self.link_board_to_token(token_name)
            else:
                print("Token placement SUCCESS.")
                self.link_board_to_token(token_name)
