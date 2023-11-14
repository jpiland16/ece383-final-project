#!/usr/bin/python3
import os

bash_rc_file = os.path.expanduser("~/.bashrc")

with open(bash_rc_file, "a+") as file:
    file.seek(0)
    lines = file.read().splitlines()
    if "source ~/catkin_ws/devel/setup.bash" in lines:
        print("Your .bashrc looks OK! Exiting...")
    else:
        print("Adding `source ~/catkin_ws/devel/setup.bash` to your .bashrc ...")
        print("source ~/catkin_ws/devel/setup.bash\n", file=file)

