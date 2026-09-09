#!/usr/bin/env bash
# =============================================================================
# 舵机投放测试：DO_SET_SERVO 驱动 SERVO9(A1)/SERVO10(A2) 按投放逻辑的 PWM 跑
# 释放(1900) → 回仓(1100) 循环，验证接线/方向/BEC 供电。
# ⚠️ DO_SET_SERVO 无需解锁即可生效（SERVOx_FUNCTION=0 前提），输出不受解锁态限制。
# 用法：bash fc_servo_test.sh          # 默认 SERVO9（A1）
#       SERVO_NUM=10 bash fc_servo_test.sh   # 测 A2
# =============================================================================
set -o pipefail
LOG=/tmp/cuadc_servo_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-/dev/cuadc-fc:115200}"
SERVO="${SERVO_NUM:-9}"
STOW=1100     # 与 mission_params.yaml servo_stowed_pwm 一致
RELEASE=1900  # 与 servo_release_pwm 一致
MPID=""; PUMP=""

ros2 daemon stop > /dev/null 2>&1 || true
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true

func_get() {  # 读 FC 参数（rcl GetParameters）
  local i out
  for i in 1 2 3; do
    out=$(timeout 25 ros2 service call /mavros/param/get_parameters \
      rcl_interfaces/srv/GetParameters "{names: ['$1']}" 2>/dev/null \
      | grep -oE "integer_value=-?[0-9]+" | head -1)
    [ -n "$out" ] && { echo "${out#integer_value=}"; return 0; }
    sleep 1
  done
  return 1
}
do_servo() {  # $1=PWM。MAV_CMD_DO_SET_SERVO=183：param1=舵机序号 param2=PWMµs
  timeout 25 ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
    "{broadcast: false, command: 183, confirmation: 0, param1: $SERVO, param2: $1, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" \
    2>/dev/null | grep -oE "success=[A-Za-z]+" | head -1
}
on_exit() {
  for p in "$PUMP" "$MPID"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  pkill -f mavros_node 2>/dev/null
}
trap on_exit EXIT INT TERM

echo "[1] mavros（$FCU）+ 心跳泵 + 参数同步（测试 SERVO$SERVO）"
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

echo "[2] 前提检查：SERVO${SERVO}_FUNCTION=$(func_get SERVO${SERVO}_FUNCTION)（应为 0）"
if [ "$(func_get SERVO${SERVO}_FUNCTION)" != "0" ]; then
  echo "  FUNCTION≠0，DO_SET_SERVO 会被功能占用拦截 → 改为 0"
  timeout 25 ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
    "{force_set: true, param_id: 'SERVO${SERVO}_FUNCTION', value: {type: 2, integer_value: 0}}" 2>/dev/null | grep -o "success=True" \
    || { echo "错误：改 FUNCTION 失败，中止"; exit 1; }
fi

echo "[2b] 解除会话级安全开关（DO_SET_SAFETY_SWITCH_STATE=1；BRD_SAFETY_DEFLT 只管开机默认）"
CMDID=$(python3 -c "from pymavlink.dialects.v20 import common as c; print(int(c.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE))")
timeout 25 ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
  "{broadcast: false, command: $CMDID, confirmation: 0, param1: 1, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" \
  2>/dev/null | grep -o "success=True" || echo "  WARN：安全开关指令未确认（继续）"
sleep 1

echo "[3] PWM 时序（收拢 $STOW / 释放 $RELEASE，留意舵机仲裁方向）"
echo "  ① 中位 1500:  $(do_servo 1500)"; sleep 2
echo "  ② 收拢 $STOW:  $(do_servo $STOW)"; sleep 2
echo "  ③ 释放 $RELEASE（投放!）: $(do_servo $RELEASE)"; sleep 1.2
echo "  ④ 回仓 $STOW:  $(do_servo $STOW)"; sleep 2
echo "  ⑤ 复投 $RELEASE: $(do_servo $RELEASE)"; sleep 1.2
echo "  ⑥ 回仓 $STOW:  $(do_servo $STOW)"; sleep 1.5
echo "  ⑦ 归中 1500:  $(do_servo 1500)"; sleep 1

echo "=== 完成，日志: $LOG ==="
echo "判读：⑤⑥ 之间舵机应两次到达释放位；若无动作依次查 BEC 5V、共地、S 针对位"
exit 0
