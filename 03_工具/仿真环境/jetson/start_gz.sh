#!/usr/bin/env bash
# Start headless Gazebo server. $1 = world file (default: physics-only flight world).
# Run AFTER start_sitl.sh (ArduPilotPlugin connection timeout expects SITL up first).
# Usage: bash ~/sim_scripts/start_gz.sh [world.sdf]
export GZ_SIM_RESOURCE_PATH="$HOME/cuadc_ws/src/cuadc_rescue_sim/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

WORLD="${1:-$HOME/sim_scripts/cuadc_rescue_flight.sdf}"

pkill -f "[c]uadc_rescue" 2>/dev/null
pkill -f "[g]z sim -s" 2>/dev/null
sleep 1

nohup gz sim -s -r -v2 "$WORLD" > /tmp/gz_sim.log 2>&1 &
echo "gz server pid=$!"
sleep 15
echo "=== /tmp/gz_sim.log (errors only) ==="
grep -iE "error|failed" /tmp/gz_sim.log | grep -viE "DynamicFactory" | head -5
echo "=== coupling: UDP 9002/9003 ==="
ss -uanp 2>/dev/null | grep -E "9002|9003"
