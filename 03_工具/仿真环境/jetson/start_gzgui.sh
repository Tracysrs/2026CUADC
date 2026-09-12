#!/usr/bin/env bash
# Attach Gazebo GUI to the running headless server (先跑 reset_sim.sh / start_gz.sh).
# 需要 Jetson 接显示器（X 在 :0）。可重复执行重挂；不动仿真本身。
# Usage: bash ~/sim_scripts/start_gzgui.sh [world.sdf]
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="$HOME/cuadc_ws/src/cuadc_rescue_sim/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

WORLD="${1:-$HOME/sim_scripts/cuadc_rescue_flight.sdf}"

setsid nohup gz sim -g "$WORLD" </dev/null > /tmp/gz_gui.log 2>&1 &
sleep 15
if timeout 5 xwininfo -root -tree 2>/dev/null | grep -q 'Gazebo Sim'; then
  echo "GUI 已挂上（$DISPLAY）。提示：右键 iris 模型 → Follow 跟机视角"
else
  echo "WARN: 15s 未见到窗口，直接重跑一次本脚本即可；排错: tail /tmp/gz_gui.log"
fi
