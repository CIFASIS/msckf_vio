#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make --pkg msckf_vio --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSAVE_TIMES=ON
