#!/usr/bin/env bash
# One-shot observability run: mavros holds 5760 while we sample everything.
source /opt/ros/humble/setup.bash
source "$HOME/cuadc_ws/install/setup.bash"
pkill -f mavros_node 2>/dev/null
sleep 1

ros2 launch mavros apm.launch fcu_url:="tcp://127.0.0.1:5760" > /tmp/mv_diag.log 2>&1 &
MPID=$!
for i in 1 2 3 4 5 6; do
  sleep 10
  alive=$(pgrep -c arducopter)
  st=$(timeout 6 ros2 topic echo --once /mavros/state 2>/dev/null | grep -m1 connected)
  echo "t=$((i*10))s ardu_alive=$alive state: $st"
done
echo "===mavros link lines==="
grep -E "opened|detected remote|VER|ERROR" /tmp/mv_diag.log | head -8
echo "===ardu.log tail==="
tail -5 /tmp/ardu.log
echo "===ardu alive final==="
pgrep -c arducopter
kill "$MPID" 2>/dev/null
pkill -f mavros_node 2>/dev/null
