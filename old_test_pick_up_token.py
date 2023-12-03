from demo_ee import MovableRobot
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

def move_cartesian_path_to_target(move_group, x: float, y: float, z: float, qx: float = 0, qy: float = 0, qz: float = 0, qw: float = 1):
    
    start_pose = move_group.get_current_pose().pose

    target_pose = Pose(
        Point(x, y, z),
        Quaternion(qx, qy, qz, qw)
    )

    waypoints = [start_pose, target_pose]

    # print(path)
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
    print("Path computation complete! Moving...")
    move_group.execute(plan)


def main():
    # robot = MovableRobot()

    # ee = robot.instance.ee_group
    # move_group = robot.instance.move_group

    # move_cartesian_path_to_target(move_group, 
    #                               x = 0.6, y = 0.007, z = 0.4)
    
    """
    Joint values directly above token 0
    [3.376873033991288, -2.0155372301770793, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]

    Pose directly above
    x = 0.6, y = 0.005, z = 0.2
    qx = 0, qy = -1, qz = 0, qw = 0


    To grab token
    z = 0.16

    Palm: wrist_3_link
    Gripper links: right_inner_finger, left_inner_finger

### Procedure
import demo_ee
robot = demo_ee.MovableRobot()
mg = robot.instance.move_group
ee = robot.instance.ee_group
jv = [3.376873033991288, -2.0155372301770793, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]
mg.go(jv)
robot.go(x = 0.62, y = 0.007, z = 0.154, qx = 0, qy = -1, qz = 0, qw = 0)
ee.go([0.5])
ee.go([0.65])

    """

def test2():
    import demo_ee
    robot = demo_ee.MovableRobot()
    mg = robot.instance.move_group
    ee = robot.instance.ee_group
    jv = [3.376873033991288, -2.0155372301770793, -1.826128842537777, -0.8930492206538982, 1.568410647973142, 1.7999299082954074]
    mg.go(jv)
    input("Press ENTER when ready")
    robot.go(x = 0.62, y = 0.007, z = 0.154, qx = 0, qy = -1, qz = 0, qw = 0)
    ee.go([0.5])
    input("Press ENTER to grab token")
    ee.go([0.65])
    input("Press ENTER to lift token")
    robot.go(z = 0.55)

if __name__ == "__main__":
    # main()
    test2()
