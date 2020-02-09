#/bin/sh
# Start alt BORIS Phase in a terminal
echo "Initiating BORIS Phase $1 (alt) in a new xterm"
BASE=`rospack find boris_master`/scripts
$BASE/xm.sh "./scripts/init_alt_$1.sh" &

