#!/usr/bin/sh
echo Configuring final project package...
echo ====================================
cd $HOME/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
echo =============================================================================
echo Configuration complete! You may need to \`source \~/catkin_ws/devel/setup.bash\`
echo in order for these scripts to work properly.
