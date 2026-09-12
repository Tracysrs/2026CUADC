#!/usr/bin/env bash
# =============================================================================
# SITL 全任务仿真：cuadc_mission 状态机全自主跑完 M1 主线
#   WAIT_FCU→…→WAIT_GUIDED→(外部切GUIDED)→WAIT_ARM(auto)→TAKEOFF→SEARCH 预设
#   航线→(m1_no_vision 旁路 ALIGN/RELEASE)→RECON_CLIMB→RECON_SURVEY 6 航点拍照
#   →RETURN_CLIMB→RETURN_HOME→LAND→DISARM→DONE
# 前置：reset_sim.sh 已起 SITL+无头GZ；要看画面另开终端挂 gz sim -g（DISPLAY=:0）
# 门禁：5760 被占即 ABORT（SERIAL0 单客户端）+ FCU 心跳稳定窗（fcu_ready.py，
#       环境变量 FCU_STAB_WINDOW=30 / FCU_READY_TIMEOUT=600 可调）
# 用法：bash ~/sim_scripts/fc_sitl_mission.sh [跟踪秒数,默认 360]
# =============================================================================
set -o pipefail
DUR="${1:-360}"
LOG=/tmp/cuadc_sitl_mission_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-tcp://127.0.0.1:5760}"

echo "[1] 启动 mavros（$FCU）"
# SERIAL0(5760) 单客户端门禁：被泄漏的 mavros/fly.py/地面站占着时，本脚本的
# mavros 必然 mode:''（TCP 连得上但拿不到流），先拦下而不是盲跑几分钟
if ss -tnp 2>/dev/null | grep -q ':5760'; then
  echo "  ABORT: 5760 已被占用（SERIAL0 单客户端）："
  ss -tnp 2>/dev/null | grep ':5760' | head -3
  exit 1
fi
ros2 launch mavros apm.launch fcu_url:="$FCU" > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 5

echo "[2] FCU 就绪门禁（直连 5762 探心跳稳定窗，跳过 reset 后不稳定窗）"
# ros2 topic echo 探连接是 DDS 假阴性老坑（09-10），弃用；改 pymavlink 直探。
# reset 后有分钟级启动不稳定窗（sketch 反复重启/时间跳变，FCU 侧
# "Accels inconsistent"+mavros "TM: Time jump"），稳定窗没过就开跑 = 必然
# mode:'' 空转全程。ABORT 后勿反复 reset——等它自愈（实测自愈）再重跑。
if ! python3 "$HOME/sim_scripts/fcu_ready.py" \
     --window "${FCU_STAB_WINDOW:-30}" --timeout "${FCU_READY_TIMEOUT:-600}"; then
  echo "  ABORT: FCU 未就绪。栈活着等自愈即可，反复 reset 会重新进不稳定窗"
  kill "$MPID" 2>/dev/null
  sleep 1
  pkill -f "mavros_node" 2>/dev/null
  exit 1
fi

echo "[3] 消息流订阅（SET_MESSAGE_INTERVAL，4.7 无 SR0 参数）"
# SYS_STATUS=1 ATTITUDE=30 LOCAL_POSITION_NED=32 GLOBAL_POSITION_INT=33
# VFR_HUD=74 BATTERY_STATUS=147 EXTENDED_SYS_STATE=245
for mid in 1 30 32 33 74 147 245; do
  timeout 8 ros2 service call /mavros/set_message_interval \
    mavros_msgs/srv/MessageInterval "{message_id: $mid, message_rate: 10.0}" \
    > /dev/null 2>&1
done
echo "  done"

echo "[4] 状态轨迹记录 + 启动任务节点（auto_arm_on_guided:=true + M1 无视觉）"
timeout $((DUR + 60)) ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/mission_states.txt" &
STPID=$!
sleep 2
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=true -p auto_arm_on_guided:=true > "$LOG/mission.log" 2>&1 &
NPID=$!

echo "[5] 外部切 GUIDED（WAIT_GUIDED 放行扳机，等效飞手确认）"
sleep 5
timeout 10 ros2 service call /mavros/set_mode \
  mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" > /dev/null 2>&1 && echo "  GUIDED 已请求"

echo "[6] 跟踪任务（每 15s 快照，DONE/节点退出即收尾）"
END=$((SECONDS + DUR))
while [ $SECONDS -lt $END ]; do
  ST=$(timeout 4 ros2 topic echo --once /cuadc/mission_state 2>/dev/null | grep -m1 'data:')
  ARMMODE=$(timeout 4 ros2 topic echo --once /mavros/state 2>/dev/null \
    | grep -E '^(mode|armed):' | tr -d ' ' | tr '\n' ' ')
  echo "  t=${SECONDS}s ${ST#data*: } | $ARMMODE"
  grep -qE '"(DONE|DISARM|ABORT)' "$LOG/mission_states.txt" 2>/dev/null && \
    { echo "  终态达成，提前收尾"; break; }
  pgrep -f "cuadc_mission_[n]ode" > /dev/null || { echo "  节点退出，提前收尾"; break; }
  sleep 15
done

echo "[7] 收尾清理"
kill "$NPID" 2>/dev/null; sleep 1
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true
kill "$STPID" 2>/dev/null
kill "$MPID" 2>/dev/null; pkill -f mavros_node 2>/dev/null
sleep 2

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹（去重计数）:"
grep -oE '"[A-Z_]+"' "$LOG/mission_states.txt" 2>/dev/null | uniq -c
echo "--- mission.log 关键行:"
grep -aE "STATE ->|任务|失败|ABORT|OVERRIDE|arm|ARM|LAND|DONE" "$LOG/mission.log" 2>/dev/null | tail -25
echo "--- mission.log 尾部:"
tail -8 "$LOG/mission.log" 2>/dev/null
