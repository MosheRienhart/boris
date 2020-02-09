#/bin/sh
# Start a BORIS Phase in a terminal (with input disabled)
echo "Initiating BORIS Phase $1 in a new xterm"
BASE=`rospack find boris_master`/scripts
$BASE/xm.sh "$BASE/init_$1.sh" &
