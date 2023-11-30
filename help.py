from object_control import *
from attacher import *
from demo_ee import *

robot = MovableRobot()
ee = robot.instance.ee_group
mg = robot.instance.move_group

adh = AttachDetachHelper()
oc = ObjectController()

from token_place_demo import *
