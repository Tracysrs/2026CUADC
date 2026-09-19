#!/usr/bin/env bash
# =============================================================================
# 舵机不动诊断：
# ① 解除会话级安全开关（MAV_CMD_DO_SET_SAFETY_SWITCH_STATE=1，§五同款）
# ② 后台抓 /mavros/rc/out，看 PWM 指令期间飞控 SERVO9/10 通道是否真在变
# 判读：rc_out 里 1100↔1600 有变化 = 飞控输出正常 → 问题在舵机侧电气
#       （BEC 5V / 共地 / S 针）；无变化 = 输出仍被门控，另有蹊跷
# =============================================================================
set -o pipefail
LOG=/tmp/cuadc_servodiag_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-/dev/cuadc-fc:115200}"
STOW=1100; RELEASE=1600  # 09-19 释放位拍板 1600
MPID=""; PUMP=""; RPID=""

ros2 daemon stop > /dev/null 2>&1 || true
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true

cmd_long() {  # $1=command $2..=param1..7，输出 success=
  timeout 25 ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
    "{broadcast: false, command: $1, confirmation: 0, param1: $2, param2: $3, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" \
    2>/dev/null | grep -oE "success=[A-Za-z]+" | head -1
}
on_exit() {
  for p in "$RPID" "$PUMP" "$MPID"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
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
for i in $(seq 1 45); do
  grep -q "parameters list received" "$LOG/mavros.log" 2>/dev/null && { echo "  参数同步（${i}s）"; break; }; sleep 1
done
ros2 daemon start > /dev/null 2>&1 || true
timeout 15 ros2 service list > /dev/null 2>&1

CMDID=$(python3 -c "from pymavlink.dialects.v20 import common as c; print(int(c.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE))")

echo "[2] 会话安全开关状态解除（command $CMDID，param1=1=关闭安全锁）"
echo "  应答: $(cmd_long $CMDID 1 0)"

echo "[3] 后台抓 /mavros/rc/out，跑一轮 PWM 时序"
timeout 40 ros2 topic echo /mavros/rc/out > "$LOG/rc_out.txt" 2>&1 &
RPID=$!
sleep 3
echo "  ① 中位 1500: $(cmd_long 183 9 1500)"; sleep 2
echo "  ② 收拢 $STOW: $(cmd_long 183 9 $STOW)"; sleep 2
echo "  ③ 释放 $RELEASE: $(cmd_long 183 9 $RELEASE)"; sleep 2
echo "  ④ 回仓 $STOW: $(cmd_long 183 9 $STOW)"; sleep 2
sleep 2

echo "[4] 分析 SERVO9（第9通道）输出值序列"
awk '/^channels:/{f=1;c=0;next} f&&/^- /{c++; if(c==9) print $2}' "$LOG/rc_out.txt" \
  | uniq -c | awk '{printf "  %s 次连续 %s V\n", $1, $2}' | sed 's/ V$//'
echo "--- SERVO10（第10通道，A2 口，对照）:"
awk '/^channels:/{f=1;c=0;next} f&&/^- /{c++; if(c==10) print $2}' "$LOG/rc_out.txt" \
  | uniq -c | head -8

echo "=== 判读 ==="
HITS=$(awk '/^channels:/{f=1;c=0;next} f&&/^- /{c++; if(c==9) print $2}' "$LOG/rc_out.txt" | sort -u | tr '\n' ' ')
echo "SERVO9 出现过的值: $HITS"
exit 0
