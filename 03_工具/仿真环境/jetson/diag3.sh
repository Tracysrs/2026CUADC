#!/usr/bin/env bash
# tcpdump capture on loopback port 5760 while mavros connects.
echo nvidia | sudo -S -p "" timeout 45 tcpdump -i lo tcp port 5760 -nn > /tmp/tcp5760.txt 2>&1 &
sleep 2
source /opt/ros/humble/setup.bash
pkill -f mavros_node 2>/dev/null
sleep 1
ros2 launch mavros apm.launch fcu_url:="tcp://127.0.0.1:5760" > /tmp/mv_diag.log 2>&1 &
MPID=$!
sleep 30
echo "===tcpdump summary==="
grep -cE "127.0.0.1" /tmp/tcp5760.txt
awk '/5760/ {print $3, $5}' /tmp/tcp5760.txt | sort | uniq -c | sort -rn | head -8
echo "===state==="
timeout 15 ros2 topic echo --once /mavros/state 2>/dev/null | grep connected
kill "$MPID" 2>/dev/null
pkill -f mavros_node 2>/dev/null
