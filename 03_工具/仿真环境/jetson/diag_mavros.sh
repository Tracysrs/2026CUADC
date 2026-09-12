#!/usr/bin/env bash
# MAVROS link diagnostic: daemon warm-up + link log + state read.
source /opt/ros/humble/setup.bash
source "$HOME/cuadc_ws/install/setup.bash"
pkill -f mavros_node 2>/dev/null
sleep 1
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
sleep 3
ros2 launch mavros apm.launch fcu_url:="tcp://127.0.0.1:5760" > /tmp/mv_diag.log 2>&1 &
MPID=$!
sleep 35
echo "===link==="
grep -E "opened|detected remote|VER" /tmp/mv_diag.log | head -6
echo "===state==="
timeout 25 ros2 topic echo --once /mavros/state 2>/dev/null | grep -E "connected|mode" | head -2
kill "$MPID" 2>/dev/null
pkill -f mavros_node 2>/dev/null
echo "===done==="
