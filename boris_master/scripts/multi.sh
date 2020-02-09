#!/bin/sh
# Launch RealSense multiple camera scan
BASE=`rospack find boris_master`/scripts
$BASE/xm.sh "roslaunch boris_scan multi.launch" &

