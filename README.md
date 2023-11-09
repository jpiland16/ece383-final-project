# Getting started
To clone and install this repository on your virtual machine, run the following
commands:

```
cd ~/catkin_ws/src
git clone git@gitlab.oit.duke.edu:jcp72/ece383-final-project.git
cd ece383-final-project
./install.sh
```

After doing this, you should be able to run the project by going to your desktop
and double-clicking the icon labeled "Launch ROS - FP".

# Modifying the environment
 > Disclaimer: I am new to this as well, so there may be a better way of doing
   this!

If you want to modify the world, you should edit `urdf/modifications.xacro` to
add lines like the following:
```xml
<xacro:include filename="$(find ece383-final-project)/urdf/YOUR_FILE_NAME.xacro"/>
<xacro:YOUR_MACRO_NAME />
```
> NOTE: `YOUR_FILE_NAME` and `YOUR_MACRO_NAME` can be the same, but they do not
have to be.

Then, make a new file in `/urdf` called `YOUR_FILE_NAME.xacro` using the
following template:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">
    <xacro:macro name="YOUR_MACRO_NAME">
        <!-- YOUR MODIFICATIONS HERE -->
    </xacro:macro>
</robot>
```
