#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum
import re

TOKEN_RE = re.compile(r"token\d+")
HOLDER_RE = re.compile(r"holder\d+")

class TokenColor(str, Enum):
    RED = "red"
    YELLOW = "yellow"

class ObjectController:

    def __init__(self) -> None:
        self._get_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self._spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self._delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    
    def delete_model_by_name(self, name: str):
        return self._delete_model(model_name=name)
    
    def delete_all_free_models(self):
        """
        Deletes all tokens and holders
        """
        self.delete_all_tokens()
        self.delete_all_holders()

    def delete_all_tokens(self):
        for model_name in self.get_model_names():
            if TOKEN_RE.match(model_name):
                self.delete_model_by_name(model_name)

    def delete_all_holders(self):
        for model_name in self.get_model_names():
            if HOLDER_RE.match(model_name):
                self.delete_model_by_name(model_name)

    def get_free_token_name(self):
        model_names = self.get_model_names()
        i = 0
        while f"token{i}" in model_names:
            i += 1
        return f"token{i}"
    
    def get_free_holder_name(self):
        model_names = self.get_model_names()
        i = 0
        while f"holder{i}" in model_names:
            i += 1
        return f"holder{i}"
    
    def _spawn_token(self, color: TokenColor, pose: Pose):
        if color not in TokenColor:
            raise ValueError(f"Invalid TokenColor: {TokenColor}")
        name = self.get_free_token_name()
        return name, self._spawn_model(
            model_name=name,
            model_xml=open(f'urdf/token_{color}.urdf', 'r').read(),
            robot_namespace='/foo', # why is this needed? -jcp72
            initial_pose=pose,
            reference_frame='world'
        )
    
    def _spawn_holder(self, pose: Pose):
        name = self.get_free_holder_name()
        return name, self._spawn_model(
            model_name=name,
            model_xml=open(f'urdf/disc_holder.urdf', 'r').read(),
            robot_namespace='/foo', # why is this needed? -jcp72
            initial_pose=pose,
            reference_frame='world'
        )
    
    def spawn_token_at_location(self, color: TokenColor, x: float, y: float, z: float):
        pose = Pose(
            Point(x, y, z),
            Quaternion(0.707, 0, 0, 0.707) # roll by pi/2 about x axis
        )
        return self._spawn_token(color, pose)
    
    def spawn_holder_at_location(self, x: float, y: float):
        pose = Pose(
            Point(x, y, 0),
            Quaternion(-0.5, 0.5, 0.5, 0.5) # pitch and yaw by pi/2 (both)
        )
        return self._spawn_holder(pose)

    def get_model_names(self):
        return self._get_properties().model_names

if __name__ == "__main__":
    pass

oc = ObjectController()
oc.delete_all_free_models()
# res = oc.delete_model_by_name("holder10")
# print(res)
res = oc.spawn_holder_at_location(0.70, 0)
print(res)
# res = oc.delete_model_by_name("token10")
# print(res)
res = oc.spawn_token_at_location(TokenColor.RED, 0.7, 0.007, 0.05)
print(res)

