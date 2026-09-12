#!/usr/bin/env bash
# Deterministic full-stack sim reset: kill everything, start fresh SITL then Gazebo.
# Usage: bash ~/sim_scripts/reset_sim.sh
pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "mavproxy.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arducopter" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
# mavros/任务节点/fly.py 也必须清：泄漏的 mavros 会自动重连并抢走 SERIAL0(5760)
# 单客户端槽，下一次任务的 mavros 就会永远 mode:''（TCP 连得上但没数据）
pkill -9 -f "mavros" 2>/dev/null
pkill -9 -f "cuadc_mission" 2>/dev/null
pkill -9 -f "fly.py" 2>/dev/null
sleep 2

# fresh SITL (fresh eeprom, gazebo-iris defaults = Quad-X 1/1)
cd "$HOME/ardupilot/Tools/autotest" || exit 1
rm -f eeprom.bin
BIN="$HOME/ardupilot/build/sitl/bin/arducopter"
DEF="$HOME/ardupilot/Tools/autotest/default_params"
# stdin keeper: SITL console reads stdin -- on ssh session close EOF would exit it
# stdbuf -oL: ardu.log 行缓冲实时可写（默认块缓冲会滞后几分钟；日志里的
# "Waiting for connection" 只是 SERIAL0 等 TCP 客户端的提示，不是 Gazebo 握手状态）
setsid bash -c "sleep infinity | stdbuf -oL '$BIN' --model JSON --speedup 1 --slave 0 --sim-address=127.0.0.1 -I0 \
  --defaults '$DEF/copter.parm,$DEF/gazebo-iris.parm,$HOME/sim_scripts/sr0.parm' > /tmp/ardu.log 2>&1" &
sleep 12

# fresh Gazebo (physics-only flight world)
export GZ_SIM_RESOURCE_PATH="$HOME/cuadc_ws/src/cuadc_rescue_sim/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"
# </dev/null: 后台进程不持有 ssh 会话 stdin，否则 ssh 不退（09-11 待办收口）
nohup gz sim -s -r -v2 "$HOME/sim_scripts/cuadc_rescue_flight.sdf" > /tmp/gz_sim.log 2>&1 < /dev/null &
sleep 15

echo "ardu=$(pgrep -c arducopter) gz=$(pgrep -c -f 'gz sim') p9002=$(ss -uanp 2>/dev/null | grep -c 9002) l5760=$(ss -tln 2>/dev/null | grep -c 5760) c5760=$(ss -tnp 2>/dev/null | grep -c ':5760')"
tail -3 /tmp/ardu.log
