#/bin/sh
# roscore not needed, roslaunch will start if necessary
BASE=`rospack find boris_master`/scripts
$BASE/xm.sh "roslaunch boris_master init.launch"

