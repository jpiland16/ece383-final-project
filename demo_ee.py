#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

"""
IMPORTANT NOTE: This code is modified only slightly from the MoveIt tutorial
available at

https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

   -- Jonathan Piland
   -- jcp72
   -- 11 November 2023
"""


import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
from math import dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from enum import Enum

always_print = print
from optional_print import optional_print as print

def disable_logging(start_with: str):

    # Uncomment the return statement below to ENABLE logging
    # return

    import rosnode
    import rosservice

    def get_node_names():
        """
        Gets a list of available services via a ros service call.
        :returns: a list of all nodes that provide the set_logger_level service, ''list(str)''
        """
        set_logger_level_nodes = []
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    set_logger_level_nodes.append(name)
        return set_logger_level_nodes
    
    def send_logger_change_message(node, logger, level):
        """
        Sends a logger level change request to 'node'.
        :param node: name of the node to chaange, ''str''
        :param logger: name of the logger to change, ''str''
        :param level: name of the level to change, ''str''
        :returns: True if the response is valid, ''bool''
        :returns: False if the request raises an exception or would not change the cached state, ''bool''
        """
        servicename = node + '/set_logger_level'

        service = rosservice.get_service_class_by_name(servicename)
        request = service._request_class()
        setattr(request, 'logger', logger)
        setattr(request, 'level', level)
        proxy = rospy.ServiceProxy(str(servicename), service)
        print(request)
        try:
            proxy(request)
        except Exception as e:
            always_print("ERROR", e)
            return False
        return True
    
    node_names = get_node_names()
    print(node_names)

    node_to_update = ""

    for node_name in node_names:
        if node_name.startswith(start_with):
            node_to_update = node_name

    if node_to_update != "":
        res = send_logger_change_message(node_to_update, "ros", "error")
        always_print("Disable logging for node", node_to_update)


class ECE383Controller(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(ECE383Controller, self).__init__()

        """
        This function obtains basic parameters needed to control the robot.
        This includes a MoveIt Commander, end effector link, planning scene,
        and the robot itself, among other things. -jcp72
        """

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ece383_fp_controller", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        disable_logging("/move_group_commander")

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander("ur5e_arm")
        ee_group = moveit_commander.MoveGroupCommander("robotiq_hand")

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.robot = robot
        self.move_group = move_group
        self.ee_group = ee_group
        self.eef_link = eef_link
        self.group_names = group_names

    def plan_cartesian_path(self, scale=1):
        """
        This function demonstrates the creation of a cartesian path and the methods for
        executing the path. However, I did not use this function in my final project.
        -jcp72
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool

    I used this function in the exact manner described above - each aspect
    of the pose is compared to its target value. Otherwise, no modifications
    were made to this function. -jcp72
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveResult(int, Enum):
    """
    This is a custom class (by jcp72) to enumerate the possible flags resulting
    from a movement attempt. Flags should be powers of 2, such that multiple
    flags can be returned, e.g. EXEC_FAILURE + POSE_NOT_ACHIEVED = 3.

    A successful move should have no flags set, i.e. = 0.
    """
    MOVE_SUCCESS = 0
    EXEC_FAILURE = 1
    POSE_NOT_ACHIEVED = 2

def move_to_pose(robot: ECE383Controller,
                 location: geometry_msgs.msg._Point.Point,
                 orientation: geometry_msgs.msg._Quaternion.Quaternion) -> int:
    """
    IMPORTANT NOTE: This function is PRIMARILY contructed from tutorial code
    available at

    https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

    with some minor modifications.

      -- Jonathan Piland
      -- jcp72
      -- 10 October 2023    
    """
    
    ret = 0

    """Get the move group from the robot class."""
    move_group = robot.move_group

    """Create a new Pose object."""
    pose_goal = geometry_msgs.msg.Pose(location, orientation)

    # Set the pose target to be that indicated.
    """Set the pose target for the robot's final position."""
    move_group.set_pose_target(pose_goal)   

    """I don't know why the tutorial repeats this."""
    move_group = robot.move_group
    ## Now, we call the planner to compute the plan and execute it.
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    if not success:
        # add the execution failure flag (jcp72 modification)
        ret += MoveResult.EXEC_FAILURE
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = move_group.get_current_pose().pose
    achieved = all_close(pose_goal, current_pose, 0.02)
    if not achieved:
        # add the pose-not-achieved flag, which indicates that the final pose
        # is not within a specified tolerance of the target pose, then add this
        # flag to the output (jcp72 modification)
        ret += MoveResult.POSE_NOT_ACHIEVED
    return ret

class MovableRobot():
    """
    (jcp72 original code)

    This is a wrapper class created by jcp72 in order to simplify some of the
    functions used in moving the robot. The `go` method provides a method to
    change just one or more of the robot parameters at a time, for example by
    calling

    ```
    MovableRobot.go(y = 0.5)
    ```

    This class was primarily used when debugging conformations using the
    interactive Python REPL (read-evaluate-print loop).
    """
    def __init__(self) -> None:
        self.instance = ECE383Controller()
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_qx = None
        self.last_qy = None
        self.last_qz = None
        self.last_qw = None
        pass
    def show_pose(self):
        # Show the current pose information.
        print(self.instance.move_group.get_current_pose().pose)
    def go(self, x = None, y = None, z = None, qx = None, qy = None, qz = None, qw = None):
        # Change one or more of the robot EE parameters
        
        # Grab the last pose if we have not previously done so
        old_pose = self.instance.move_group.get_current_pose().pose
        if self.last_x is None:
            self.last_x = old_pose.position.x
        if self.last_y is None:
            self.last_y = old_pose.position.y
        if self.last_z is None:
            self.last_z = old_pose.position.z
        if self.last_qx is None:
            self.last_qx = old_pose.orientation.x
        if self.last_qy is None:
            self.last_qy = old_pose.orientation.y
        if self.last_qz is None:
            self.last_qz = old_pose.orientation.z
        if self.last_qw is None:
            self.last_qw = old_pose.orientation.w

        # Set parameters if not present
        if x is None:
            x = self.last_x
        if y is None:
            y = self.last_y
        if z is None:
            z = self.last_z
        if qx is None:
            qx = self.last_qx
        if qy is None:
            qy = self.last_qy
        if qz is None:
            qz = self.last_qz
        if qw is None:
            qw = self.last_qw

        # Store updates
        self.last_x = x
        self.last_y = y
        self.last_z = z
        self.last_qx = qx
        self.last_qy = qy
        self.last_qz = qz
        self.last_qw = qw

        result = move_to_pose(self.instance, 
                            geometry_msgs.msg.Point(x, y, z),
                            geometry_msgs.msg.Quaternion(qx, qy, qz, qw))
        
        print("New pose: ")
        print(self.instance.move_group.get_current_pose().pose)

        success = True
        # check for error flags
        for flag in list(MoveResult):
            if result & flag:
                success = False
                print("WARNING: result flag: " + str(flag))
        if success:
            print("Move successful!")

def test():
    ee = MovableRobot().instance.ee_group
    ee.go([0.5])


if __name__ == "__main__":
    disable_logging()