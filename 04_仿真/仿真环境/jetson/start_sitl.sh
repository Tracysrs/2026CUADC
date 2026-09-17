#!/usr/bin/env bash
# =============================================================================
# Start ArduCopter SITL directly (no sim_vehicle / MAVProxy layer).
# Links: TCP 5760 (SERIAL0, MAVROS), 5762/5763; Gazebo JSON via ArduPilotPlugin.
# Fresh eeprom each start: params = copter.parm + gazebo-iris.parm (Quad-X).
# Usage: bash ~/sim_scripts/start_sitl.sh
# =============================================================================
export PATH="$HOME/.local/bin:$PATH"

pkill -f "[s]im_vehicle.py" 2>/dev/null
pkill -f "[b]uild/sitl/bin/arducopter" 2>/dev/null
pkill -f "[m]avproxy.py" 2>/dev/null
sleep 1

cd "$HOME/ardupilot/Tools/autotest" || exit 1
rm -f eeprom.bin

BIN="$HOME/ardupilot/build/sitl/bin/arducopter"
DEF="$HOME/ardupilot/Tools/autotest/default_params"
# setsid: detach from ssh session so it survives session close (KillUserProcesses)
# stdin keeper: SITL console reads stdin -- on ssh session close EOF would exit it
#   （"幽灵退出"真凶，与 reset_sim.sh 同款；2026-09-12 深夜实证 start_sitl 漏配）
# stdbuf -oL: ardu.log 行缓冲实时可写
# sr0.parm: SERIAL0 流请求（与 reset_sim.sh 一致）
setsid bash -c "sleep infinity | stdbuf -oL '$BIN' --model JSON --speedup 1 --slave 0 --sim-address=127.0.0.1 -I0 \
  --defaults '$DEF/copter.parm,$DEF/gazebo-iris.parm,$HOME/sim_scripts/sr0.parm' > /tmp/ardu.log 2>&1" &
echo "arducopter pid=$!"
sleep 15
echo "=== /tmp/ardu.log tail ==="
tail -4 /tmp/ardu.log
echo "arducopter_procs=$(pgrep -c arducopter)"
