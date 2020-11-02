#!/bin/bash
source /opt/ros/kinetic/setup.bash
catkin_make --pkg msckf_vio --cmake-args -DCMAKE_BUILD_TYPE=Release
