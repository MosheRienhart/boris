#!/bin/sh
# Start a keyboard teleop suitable for the BORIS
echo "Starting keyboard teleop"
BASE=`rospack find boris_master`/scripts
$BASE/xt.sh "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
