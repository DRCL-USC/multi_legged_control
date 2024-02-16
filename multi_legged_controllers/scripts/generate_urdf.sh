#!/usr/bin/env sh
mkdir -p /tmp/legged_control/
rosrun xacro xacro $1 robot_type:=$2 ns:=$3 > /tmp/legged_control/$3.urdf
