#!/bin/sh
# Launch Phase 1 of the BORIS software stack
echo "BORIS Phase 1: Realsense camera and laser scan emulation"
roslaunch boris_master init_1.launch
