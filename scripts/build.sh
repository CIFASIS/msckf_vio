#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make --pkg msckf_vio --cmake-args -DCMAKE_BUILD_TYPE=Release -DSAVE_TIMES=ON
