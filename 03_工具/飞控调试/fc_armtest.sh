#!/usr/bin/env bash
# =============================================================================
# 真飞控直接解锁验证：绕过任务状态机（无 GPS 时其 WAIT_NAV_STABLE 永不过），
# 直接 CommandBool 解锁 10s → 上锁。⚠️ 无桨桌面作业；MOT_SPIN_ARM=0.1 电机会怠速。
# 参数临时放宽 + 恢复逻辑与 fc_armrun.sh 相同（trap 兜底）。
# =============================================================================
set -o pipefail
LOG=/tmp/cuadc_armtest_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-/dev/cuadc-fc:115200}"
CHANGED=0
MPID=""; PUMP=""; STPID=""

ros2 daemon stop > /dev/null 2>&1 || true
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true

param_get() {
  local i out
  for i in 1 2 3; do
    out=$(timeout 25 ros2 service call /mavros/param/get_parameters \
      rcl_interfaces/srv/GetParameters "{names: ['$1']}" 2>/dev/null \
      | grep -oE "integer_value=-?[0-9]+" | head -1)
    [ -n "$out" ] && { echo "integer: ${out#integer_value=}"; return 0; }
    sleep 1
  done
  return 1
}
param_set() {
  local i ok
  for i in 1 2; do
    ok=$(timeout 25 ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
      "{force_set: true, param_id: '$1', value: {type: 2, integer_value: $2}}" 2>/dev/null \
      | grep -c "success=True")
    if [ "$ok" -ge 1 ]; then sleep 2; return 0; fi
    sleep 1
  done
  return 1
}
restore_params() {
  param_set ARMING_SKIPCHK 0
  param_set FS_THR_ENABLE 5
  echo "[恢复校验] SKIPCHK=$(param_get ARMING_SKIPCHK)  THR=$(param_get FS_THR_ENABLE)"
}
on_exit() {
  if [ "$CHANGED" = "1" ]; then restore_params; fi
  for p in "$STPID" "$PUMP" "$MPID"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  pkill -f "cuadc_mission_[n]ode" 2>/dev/null
  pkill -f mavros_node 2>/dev/null
}
trap on_exit EXIT INT TERM

echo "[1] mavros + 心跳泵 + 参数同步"
ros2 launch mavros apm.launch fcu_url:="$FCU" gcs_url:=tcp-l://0.0.0.0:14550 \
  > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 2
nohup python3 "$HOME/gcs_pump.py" > "$LOG/pump.log" 2>&1 &
PUMP=$!
ok=""
for i in $(seq 1 30); do
  grep -q "Got HEARTBEAT" "$LOG/mavros.log" 2>/dev/null && { ok=1; break; }; sleep 1
done
[ -n "$ok" ] || { echo "错误：未连上飞控"; exit 1; }
ok=""
for i in $(seq 1 45); do
  grep -q "parameters list received" "$LOG/mavros.log" 2>/dev/null && { echo "  参数同步（${i}s）"; ok=1; break; }; sleep 1
done
[ -n "$ok" ] || echo "  WARN：参数列表 45s 未同步完"
ros2 daemon start > /dev/null 2>&1 || true
timeout 15 ros2 service list > /dev/null 2>&1

timeout 120 ros2 topic echo /mavros/statustext/recv > "$LOG/statustext.txt" 2>&1 &
STPID=$!

echo "[2] 放宽解锁检查"
param_set ARMING_SKIPCHK -1 && param_set FS_THR_ENABLE 0 && CHANGED=1
[ "$(param_get ARMING_SKIPCHK)" = "integer: -1" ] || { echo "错误：放宽未生效"; exit 1; }
echo "  ARMING_SKIPCHK=-1 / FS_THR_ENABLE=0 已生效"

echo "[3] 切 GUIDED → 解锁"
timeout 25 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}" > "$LOG/setmode.txt" 2>&1
grep -o "mode_sent=True" "$LOG/setmode.txt" || echo "WARN: mode 切换未确认"
timeout 25 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}" > "$LOG/arm.txt" 2>&1
grep -oE "success=[A-Za-z]+|result=[A-Z_]+" "$LOG/arm.txt" | tr '\n' ' '; echo ""
sleep 10
ros2 daemon stop > /dev/null 2>&1 || true
timeout 10 ros2 topic echo --once /mavros/state > "$LOG/state_armed.txt" 2>&1
grep -E "armed|guided|mode:" "$LOG/state_armed.txt"

echo "[4] 上锁"
timeout 25 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: false}" > "$LOG/disarm.txt" 2>&1
grep -oE "success=[A-Za-z]+|result=[A-Z_]+" "$LOG/disarm.txt" | tr '\n' ' '; echo ""
sleep 2
timeout 10 ros2 topic echo --once /mavros/state --field armed 2>/dev/null | tail -1

echo "[5] 恢复参数"
if [ "$(param_get ARMING_SKIPCHK)" != "integer: 0" ] || [ "$(param_get FS_THR_ENABLE)" != "integer: 5" ]; then
  restore_params
  [ "$(param_get ARMING_SKIPCHK)" = "integer: 0" ] && [ "$(param_get FS_THR_ENABLE)" = "integer: 5" ] && CHANGED=0
else
  CHANGED=0
fi
[ "$CHANGED" = "0" ] && echo "  参数已恢复原值（SKIPCHK=0 / THR=5）✅" || echo "  ⚠️ 恢复未确认，退出时 trap 再试"

echo "=== 完成，日志: $LOG ==="
echo "--- 解锁相关 STATUSTEXT:"; grep -iE "text:.*([Aa]rm|GUIDED|safe)" "$LOG/statustext.txt" 2>/dev/null | tail -10
exit 0
