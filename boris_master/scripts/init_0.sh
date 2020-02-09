#!/bin/sh
# Launch phase 0 of the BORIS software stack
echo "BORIS Phase 0: Motor control and odometry"
roslaunch boris_master init_0.launch
