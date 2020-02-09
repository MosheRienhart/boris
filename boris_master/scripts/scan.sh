#!/bin/sh
# Launch RealSense scan
BASE=`rospack find boris_master`/scripts
$BASE/xm.sh "roslaunch boris_scan scan.launch" &

