#!/usr/bin/sh
echo Configuring final project package...
echo ====================================
cd $HOME
# install connect4 solver package
if [ ! -d connect4 ]; then
  # directory does NOT exist - need to clone the repo from GitHub
  git clone https://github.com/PascalPons/connect4
else
  echo You already have the \`connect4\` package installed.
fi
cd connect4
# compile the solver program and copy to this repo
make c4solver
cd $HOME/catkin_ws/src/ece383-final-project
cp $HOME/connect4/c4solver .
# download book
wget https://github.com/PascalPons/connect4/releases/download/book/7x6.book
cd $HOME/catkin_ws/src
# install roboticsgroup gazebo plugins (needed for MimicJoint)
if [ ! -d roboticsgroup_gazebo_plugins ]; then
  # directory does NOT exist - need to clone the repo from GitHub
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
else
  echo You already have the \`roboticsgroup_gazebo_plugins\` package installed.
fi
# install attacher plugin (needed to pick up objects)
if [ ! -d gazebo_ros_link_attacher ]; then
  # directory does NOT exist - need to clone the repo from GitHub
  git clone https://github.com/pal-robotics/gazebo_ros_link_attacher
else
  echo You already have the \`gazebo_ros_link_attacher\` package installed.
fi
# install gazebo-pkgs - previously used for GraspFix but now unused
if [ ! -d gazebo-pkgs ]; then
  # directory does NOT exist - need to clone the repo from GitHub
  git clone https://github.com/JenniferBuehler/gazebo-pkgs
  cd gazebo-pkgs
  rm -rf gazebo_state_plugins # dependency not found, and we don't need it
  rm -rf gazebo_test_tools    # dependency not found, and we don't need it
  cd ..
else
  echo You already have the \`gazebo-pkgs\` package installed.
fi
cd $HOME/catkin_ws
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
