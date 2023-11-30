import rospy
from gazebo_ros_link_attacher.srv import Attach

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

