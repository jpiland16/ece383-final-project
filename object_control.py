#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetModelState, SetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import JointState
from forward_kinematics import get_ee_pos

import tf.transformations as tr
import numpy as np

from enum import Enum
import re

TOKEN_RE = re.compile(r"token\d+")
HOLDER_RE = re.compile(r"holder\d+")

class JointStatesNotAvailableError(ValueError):
    pass
class JointStatesWrongLengthError(ValueError):
    pass

class TokenColor(str, Enum):
    RED = "red"
    YELLOW = "yellow"

class ObjectController:

    def __init__(self) -> None:
        self._get_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self._get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self._delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self._joint_states = None
        rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
    
    def delete_model_by_name(self, name: str):
        return self._delete_model(model_name=name)
    
    def get_robot_ee_pose(self):
        if self._joint_states is None:
            raise JointStatesNotAvailableError("Joint states not yet available!")
        if len(self._joint_states) != 7:
            raise JointStatesWrongLengthError(
                f"Expected 7 joint values, got {len(self._joint_states)}")
        
        position = self._joint_states[:] # deep copy (drop reference)

        position = [position[0]] + position[2:] # drop finger joint (index 1)

        modified_pos = position[:] # independent copy
        modified_pos[0] = position[2] # two joints are backwards
        modified_pos[0] += np.pi
        modified_pos[2] = position[0]

        A = get_ee_pos(modified_pos) # transformation to EE
        x = A[0][3]
        y = A[1][3]
        z = A[2][3]

        position = Point(x, y, z)
        orientation: Quaternion = Quaternion(*(tr.quaternion_from_matrix(A) * -1))
        return Pose(position, orientation)
        
    def _joint_state_callback(self, msg: JointState):
        self._joint_states = list(msg.position)

    def get_model_state(self, model_name: str) -> GetModelStateResponse:
        return self._get_model_state(model_name, "")
    
    def edit_model_state(self, model_name: str, x: float = None, y: float = None, z: float = None, qx: float = None, qy: float = None, qz: float = None, qw: float = None):

        if None in [x, y, z, qx, qy, qz, qw]:
            old_state = self.get_model_state(model_name)

        if x is None:
            x = old_state.pose.position.x
        if y is None:
            y = old_state.pose.position.y
        if z is None:
            z = old_state.pose.position.z
        if qx is None:
            qx = old_state.pose.orientation.x
        if qy is None:
            qy = old_state.pose.orientation.y
        if qz is None:
            qz = old_state.pose.orientation.z
        if qw is None:
            qw = old_state.pose.orientation.w

        pos = Point(x, y, z)
        ori = Quaternion(qx, qy, qz, qw)
        state = ModelState(model_name, Pose(pos, ori), Twist(), "")
        return self._set_model_state(state)
    
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
    
    def spawn_holder_at_location(self, x: float, y: float, z: float = 0):
        pose = Pose(
            Point(x, y, z),
            Quaternion(-0.5, 0.5, 0.5, 0.5) # pitch and yaw by pi/2 (both)
        )
        return self._spawn_holder(pose)

    def get_model_names(self):
        return self._get_properties().model_names

def set_up_staged_tokens(oc: ObjectController):
    # create 7-by-3 array of tokens and holders (so they stay still)
    for i in range(7): # x index
        for j in range(3): # y index
            x = 0.6 + 0.05 * i
            y = 0.0 + 0.05 * j
            # oc.spawn_holder_at_location(x, y)
            oc.spawn_token_at_location(TokenColor.RED, x + 0.016, y + 0.007, 0.10)
            return

def drop_token_in_column(oc: ObjectController, color: TokenColor, column: int):
    if (not (1 <= column <= 7)) or (int(column) != column):
        raise ValueError(f"Invalid column id: {column}")
    
    # x_pos = [0.518, 0.551, 0.585, 0.620, 0.655, 0.690, 0.725][column - 1]
    x_pos = 0.204 + 0.0348 * (column - 1)

    return oc.spawn_token_at_location(color, x_pos, -0.2976, 0.38)

def oj_setup():
    oc = ObjectController()
    oc.delete_all_free_models()
    set_up_staged_tokens(oc)
    return oc

def main():
    oc = ObjectController()
    oc.edit_model_state("token0", z = 0.5)
    # oc = oj_setup()
    return
    drop_token_in_column(oc, TokenColor.RED, 1)
    drop_token_in_column(oc, TokenColor.YELLOW, 2)
    drop_token_in_column(oc, TokenColor.RED, 3)
    drop_token_in_column(oc, TokenColor.YELLOW, 4)
    drop_token_in_column(oc, TokenColor.RED, 5)
    drop_token_in_column(oc, TokenColor.YELLOW, 6)
    drop_token_in_column(oc, TokenColor.RED, 7)
    return
    counter = 1
    players = [TokenColor.RED, TokenColor.YELLOW]
    while True:
        col = input("Enter column (1 - 7) to play (-1 to exit) >>> ")
        try:
            col = int(col)
            if col == -1:
                break
            drop_token_in_column(oc, players[counter % 2], col)
            counter += 1
        except:
            print("ERROR!")
            pass

if __name__ == "__main__":
    main()

# oc = ObjectController()
# oc.delete_all_free_models()
# # res = oc.delete_model_by_name("holder10")
# # print(res)
# res = oc.spawn_holder_at_location(0.70, 0)
# print(res)
# # res = oc.delete_model_by_name("token10")
# # print(res)
# res = oc.spawn_token_at_location(TokenColor.RED, 0.7, 0.007, 0.05)
# print(res)

