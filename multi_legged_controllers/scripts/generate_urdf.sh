#!/usr/bin/env sh
mkdir -p /tmp/legged_control/
rosrun xacro xacro $1 robot_type:=$2 ns:=$3 prefix:=$4 sim:=$5> /tmp/legged_control/$3.urdf
