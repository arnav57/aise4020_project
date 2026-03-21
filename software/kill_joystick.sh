#!/bin/bash
echo "--- Stopping ChassisServer ---"
supervisorctl stop ChassisServer

echo "--- Nuking ROS 2 Processes ---"
# This kills anything with 'joy' or 'launch' in the command line
pkill -9 -f joy
pkill -9 -f launch

echo "--- Cleaning ROS 2 Daemon Cache ---"
ros2 daemon stop
ros2 daemon start

echo "--- Final Check ---"
ros2 node list

echo "--- Installing Python Pkgs ---"
pip install flask