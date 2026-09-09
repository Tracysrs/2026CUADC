#!/usr/bin/env bash
# =============================================================================
# 真飞控解锁试跑 v2：临时放宽解锁检查 → 状态机全流程（自动解锁→起飞尝试）→ 恢复
# ⚠️ 仅限无桨桌面作业。MOT_SPIN_ARM=0.1：解锁后电机会 10% 怠速，属正常。
# 临时参数：ARMING_SKIPCHK=-1（跳过全部解锁检查，源码实证的迁移语义）+
#           FS_THR_ENABLE=0（无遥控不触发 RTL 失效保护）
# 教训固化：① mavros 参数列表同步(~10s+)完成前，rcl 镜像读出的是陈旧默认值，
#   set 也会失败——必须等 "parameters list received"；② set 必须校验 success 并
#   回读；③ 任何退出路径都恢复参数（trap 兜底）。
# 用法：bash fc_armrun.sh [试跑秒数，默认 120]
# =============================================================================
set -o pipefail
RUN="${1:-120}"
LOG=/tmp/cuadc_armrun_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-/dev/cuadc-fc:115200}"
CHANGED=0
NPID=""; MPID=""; PUMP=""; STPID=""

ros2 daemon stop > /dev/null 2>&1 || true
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true

param_get() {  # 输出 "integer: N"。此版 mavros 无 /mavros/param/get，
  # 走 rcl 风格 /mavros/param/get_parameters；重试 3 次
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
param_set() {  # ParamSetV2：force_set=true 直发 FCU（value.type=2=整数）；校验 success+重试。
  # 注意：此版 mavros 的 /mavros/param/set 已改为 ParamSetV2（旧 ParamSet 类型不存在）
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
  echo "[恢复] ARMING_SKIPCHK → 0，FS_THR_ENABLE → 5"
  param_set ARMING_SKIPCHK 0
  param_set FS_THR_ENABLE 5
  echo "[恢复校验] SKIPCHK=$(param_get ARMING_SKIPCHK)  THR=$(param_get FS_THR_ENABLE)"
}
on_exit() {  # 唯一出口：参数恢复兜底 + 全进程清理
  if [ "$CHANGED" = "1" ]; then restore_params; fi
  for p in "$NPID" "$STPID" "$PUMP" "$MPID"; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  pkill -f "cuadc_mission_[n]ode" 2>/dev/null
  pkill -f mavros_node 2>/dev/null
}
trap on_exit EXIT INT TERM

echo "[1] mavros（$FCU）+ GCS 心跳泵 + 等参数列表同步"
ros2 launch mavros apm.launch fcu_url:="$FCU" gcs_url:=tcp-l://0.0.0.0:14550 \
  > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 2
nohup python3 "$HOME/gcs_pump.py" > "$LOG/pump.log" 2>&1 &
PUMP=$!
ok=""
for i in $(seq 1 30); do
  grep -q "Got HEARTBEAT" "$LOG/mavros.log" 2>/dev/null && { ok=1; echo "  HEARTBEAT（${i}s）"; break; }
  sleep 1
done
[ -n "$ok" ] || { echo "错误：未连上飞控"; exit 1; }
ok=""
for i in $(seq 1 45); do
  grep -q "parameters list received" "$LOG/mavros.log" 2>/dev/null && { ok=1; echo "  参数列表同步完成（${i}s）"; break; }
  sleep 1
done
[ -n "$ok" ] && sleep 2 || echo "  WARN：45s 未同步完参数列表（继续，读回校验会兜底）"
# 预热 ros2-daemon + 服务发现（冷启动首调可能报 context invalid，9-9 实坑）
ros2 daemon start > /dev/null 2>&1 || true
timeout 15 ros2 service list > /dev/null 2>&1

echo "[2] 记录原参数 → 放宽解锁条件"
param_get ARMING_SKIPCHK > "$LOG/orig_arming_skipchk.txt" ||
  { echo "错误：读不到原参数"; exit 1; }
param_get FS_THR_ENABLE > "$LOG/orig_fs_thr.txt"
echo "  原 ARMING_SKIPCHK=$(cat "$LOG/orig_arming_skipchk.txt")  FS_THR_ENABLE=$(cat "$LOG/orig_fs_thr.txt")"
param_set ARMING_SKIPCHK -1 && param_set FS_THR_ENABLE 0 && CHANGED=1
echo "  放宽读回 ARMING_SKIPCHK=$(param_get ARMING_SKIPCHK)（应 -1）  FS_THR_ENABLE=$(param_get FS_THR_ENABLE)（应 0）"
[ "$(param_get ARMING_SKIPCHK)" = "integer: -1" ] || { echo "错误：放宽未生效，中止"; exit 1; }

echo "[3] 切 GUIDED"
timeout 25 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}" > "$LOG/setmode.txt" 2>&1
grep -o "mode_sent=.*" "$LOG/setmode.txt" | head -1
sleep 2

echo "[4] 任务状态机试跑 ${RUN}s（自动解锁→起飞尝试；无桨，电机将怠速）"
timeout $((RUN + 10)) ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/mission_states.txt" &
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=true -p auto_arm_on_guided:=true > "$LOG/mission.log" 2>&1 &
NPID=$!
sleep "$RUN"

echo "[5] 上锁检查"
ros2 daemon stop > /dev/null 2>&1 || true
timeout 10 ros2 topic echo --once /mavros/state > "$LOG/state_end.txt" 2>&1
if grep -q "armed: true" "$LOG/state_end.txt"; then
  echo "  仍处于解锁 → 上锁"
  timeout 25 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    "{value: false}" > "$LOG/disarm.txt" 2>&1
  sleep 2
  timeout 10 ros2 topic echo --once /mavros/state --field armed 2>/dev/null | tail -1
else
  echo "  已在锁定状态"
fi
pkill -f "cuadc_mission_[n]ode" 2>/dev/null

echo "[6] 恢复参数"
if [ "$(param_get ARMING_SKIPCHK)" != "integer: 0" ] || [ "$(param_get FS_THR_ENABLE)" != "integer: 5" ]; then
  restore_params
  [ "$(param_get ARMING_SKIPCHK)" = "integer: 0" ] && [ "$(param_get FS_THR_ENABLE)" = "integer: 5" ] && CHANGED=0
else
  CHANGED=0
  echo "  参数已是原值"
fi
[ "$CHANGED" = "0" ] && echo "  参数恢复 ✅（SKIPCHK=0 / THR=5）" || echo "  ⚠️ 恢复未确认，退出时 trap 会再试"

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹:"; grep -c . "$LOG/mission_states.txt" 2>/dev/null; cat "$LOG/mission_states.txt" 2>/dev/null | head -30
echo "--- mission.log 关键行:"; grep -E "STATE ->|任务|失败|ABORT|OVERRIDE|[Aa]rm" "$LOG/mission.log" 2>/dev/null | tail -25
echo "--- STATUSTEXT 关键行:"; grep -iE "text:|[Aa]rm" "$LOG/statustext.txt" 2>/dev/null | tail -20
exit 0
