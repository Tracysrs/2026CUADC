#!/usr/bin/env bash
# =============================================================================
# SITL 干跑：cuadc_mission 状态机对 Gazebo+SITL 推进（SITL 有仿真 GPS，应过
# WAIT_NAV_STABLE）。安全约束：不解锁（auto_arm 默认 false）。
# 前置：start_sitl.sh 已带 --out=udp:127.0.0.1:14551（MAVROS 专用出口）
# 用法：bash ~/sim_scripts/fc_sitl_dryrun.sh [干跑秒数，默认 120]
# =============================================================================
set -o pipefail
DUR="${1:-120}"
LOG=/tmp/cuadc_sitl_dryrun_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-tcp://127.0.0.1:5760}"

echo "[1] 启动 mavros（$FCU）"
ros2 launch mavros apm.launch fcu_url:="$FCU" > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 5

echo "[2] 等连接（40s）"
ok=""
for i in $(seq 1 20); do
  if timeout 3 ros2 topic echo --once /mavros/state --field connected 2>/dev/null | grep -q true; then
    ok=1; echo "  CONNECTED（第 ${i} 次探测）"; break
  fi
  sleep 2
done
[ -n "$ok" ] || echo "  WARN: 40s 未确认连接（继续，靠日志诊断）"

# STATUSTEXT 全程记录
timeout $((DUR + 150)) ros2 topic echo /mavros/statustext/recv > "$LOG/statustext.txt" 2>&1 &
STPID=$!
sleep 2

echo "[3] 遥测快照"
timeout 10 ros2 topic echo --once /mavros/global_position/global > "$LOG/global.txt" 2>&1
timeout 10 ros2 topic echo --once /mavros/local_position/pose > "$LOG/local_pose.txt" 2>&1
timeout 8  ros2 topic echo --once /mavros/battery > "$LOG/battery.txt" 2>&1

echo "[3.5] 消息流订阅（SET_MESSAGE_INTERVAL，4.7 无 SR0 参数）"
# SYS_STATUS=1 ATTITUDE=30 LOCAL_POSITION_NED=32 GLOBAL_POSITION_INT=33
# VFR_HUD=74 BATTERY_STATUS=147 EXTENDED_SYS_STATE=245
for mid in 1 30 32 33 74 147 245; do
  timeout 8 ros2 service call /mavros/set_message_interval \
    mavros_msgs/srv/MessageInterval "{message_id: $mid, message_rate: 10.0}" \
    > /dev/null 2>&1
done
echo "  done"

echo "[4] 任务状态机干跑 ${DUR}s（不解锁）"
timeout $((DUR + 10)) ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/mission_states.txt" &
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=true > "$LOG/mission.log" 2>&1 &
NPID=$!
timeout 20 ros2 topic hz /mavros/local_position/odom > "$LOG/odom_hz.txt" 2>&1 &
timeout 20 ros2 topic hz /mavros/local_position/pose > "$LOG/pose_hz.txt" 2>&1 &
sleep "$DUR"

echo "[5] 收尾清理"
echo "--- odom/pose 频率:"
tail -2 "$LOG/odom_hz.txt" 2>/dev/null
tail -2 "$LOG/pose_hz.txt" 2>/dev/null
kill "$NPID" 2>/dev/null; sleep 1
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true
kill "$STPID" 2>/dev/null
kill "$MPID" 2>/dev/null; pkill -f mavros_node 2>/dev/null
sleep 2

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹（mission_states）:"; tail -30 "$LOG/mission_states.txt" 2>/dev/null
echo "--- mission.log 状态行:"; grep -E "STATE ->|任务|失败|ABORT|OVERRIDE" "$LOG/mission.log" 2>/dev/null | tail -20
echo "--- mission.log 尾部:"; tail -8 "$LOG/mission.log" 2>/dev/null
echo "--- 遥测: global.txt 首行坐标"; grep -m2 -E "latitude|longitude|altitude" "$LOG/global.txt" 2>/dev/null
