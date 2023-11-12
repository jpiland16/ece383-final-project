#!/usr/bin/sh
echo Configuring final project package...
echo ====================================
cd $HOME/catkin_ws
# install roboticsgroup gazebo plugins (needed for MimicJoint)
if [ ! -d /path/to/directory ]; then
  # directory does NOT exist - need to clone the repo from GitHub
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
cd $HOME/catkin_ws/src/ece383-final-project
#### Create desktop shortcut:
sed s@USER_HOME@$HOME@ launch_ros_fp.desktop.template > ~/Desktop/launch_ros_fp.desktop
chmod 755 ~/Desktop/launch_ros_fp.desktop
echo =============================================================================
echo Configuration complete! You may need to \`source \~/catkin_ws/devel/setup.bash\`
echo in order for these scripts to work properly.
