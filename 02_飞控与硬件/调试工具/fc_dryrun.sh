#!/usr/bin/env bash
# =============================================================================
# 真飞控干跑：CUAV V6X 经 USB(/dev/cuadc-fc) 接 Jetson，验证链路 + 状态机推进
# 安全约束：无桨作业；不发送任何 arm 指令（auto_arm 默认 false，仅观察 PreArm）
# 用法：bash fc_dryrun.sh [干跑秒数，默认 90]   日志：/tmp/cuadc_fc_dryrun_*/
# =============================================================================
# set -u 不可用（同 m1_sitl_accept.sh）：ROS 2 setup.bash 引用未绑定变量
set -o pipefail
DUR="${1:-90}"
LOG=/tmp/cuadc_fc_dryrun_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-/dev/cuadc-fc:115200}"

echo "[0] 设备: $(ls -l /dev/cuadc-fc 2>/dev/null || echo 缺失)"
[ -e /dev/cuadc-fc ] || { echo "错误：/dev/cuadc-fc 不存在"; exit 1; }

echo "[1] 启动 mavros（$FCU）+ GCS 心跳泵（tcp-l:14550）"
ros2 launch mavros apm.launch fcu_url:="$FCU" gcs_url:=tcp-l://0.0.0.0:14550 \
  > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 3
nohup python3 "$HOME/gcs_pump.py" > "$LOG/pump.log" 2>&1 &
PUMP=$!

echo "[2] 等连接（30s）"
ok=""
for i in $(seq 1 15); do
  if timeout 3 ros2 topic echo --once /mavros/state --field connected 2>/dev/null | grep -q true; then
    ok=1; echo "  CONNECTED（第 ${i} 次探测）"; break
  fi
  sleep 2
done
[ -n "$ok" ] || echo "  WARN: 30s 未确认连接（继续，靠日志诊断）"

# STATUSTEXT 全程记录（PreArm/报错都在这里），先挂上避免漏早期报文
timeout $((DUR + 150)) ros2 topic echo /mavros/statustext/recv > "$LOG/statustext.txt" 2>&1 &
STPID=$!
sleep 2

echo "[3] 遥测快照"
timeout 8  ros2 topic echo --once /mavros/battery > "$LOG/battery.txt" 2>&1
timeout 8  ros2 topic echo --once /mavros/vfr_hud   > "$LOG/vfr_hud.txt" 2>&1
timeout 10 ros2 topic echo --once /diagnostics      > "$LOG/diagnostics.txt" 2>&1

echo "[4] 模式切换测试 → GUIDED"
timeout 15 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}" > "$LOG/setmode.txt" 2>&1
sleep 2
timeout 5 ros2 topic echo --once /mavros/state > "$LOG/state_after_setmode.txt" 2>&1

echo "[5] 任务状态机干跑 ${DUR}s（无桨，不解锁）"
timeout $((DUR + 10)) ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/mission_states.txt" &
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=true > "$LOG/mission.log" 2>&1 &
NPID=$!
sleep "$DUR"

echo "[6] 收尾清理"
kill "$NPID" 2>/dev/null; sleep 1
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true
kill "$STPID" "$PUMP" 2>/dev/null
kill "$MPID" 2>/dev/null; pkill -f mavros_node 2>/dev/null
sleep 2

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹（mission_states）:"; tail -30 "$LOG/mission_states.txt" 2>/dev/null
echo "--- mission.log 状态行:"; grep -E "STATE ->|任务|失败|ABORT|OVERRIDE" "$LOG/mission.log" 2>/dev/null | tail -20
echo "--- mission.log 尾部:"; tail -10 "$LOG/mission.log" 2>/dev/null
echo "--- STATUSTEXT（PreArm/告警）:"; grep -B1 -A1 -iE "text:|prearm|critical|error" "$LOG/statustext.txt" 2>/dev/null | tail -30
