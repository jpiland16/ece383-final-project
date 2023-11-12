import os, sys

print("STARTING FINAL PROJECT...")

commands = [
    "roslaunch ece383-final-project ur5e_bringup.launch",
    "sleep 10 ; roslaunch ece383-final-project moveit_planning_execution.launch sim:=true"
]

if len(sys.argv) == 1:
    # try to launch all commands
    for command in commands:
        print("LAUNCH: " + command)
        os.system(f"bash -c \"source $HOME/ws_moveit/devel/setup.bash ; source $HOME/catkin_ws/devel/setup.bash ; cd $HOME/catkin_ws/src ; {command} &\"")
        # os.system(f'xfce4-terminal -e \'bash -c "source /home/jcp72/ws_moveit/devel/setup.bash ; cd /home/jcp72/catkin_ws/src ; {command}"\'')
elif len(sys.argv) == 2:
    # launch an individual command
    i = int(sys.argv[1])
    print(commands[i])
    os.system(f"bash -c \"source $HOME/ws_moveit/devel/setup.bash ; source $HOME/catkin_ws/devel/setup.bash ; cd $HOME/catkin_ws/src ; {commands[i]}\"")
else:
    print("Wrong number of arguments... press ENTER to exit")
    input()

print("Goodbye")
input()
