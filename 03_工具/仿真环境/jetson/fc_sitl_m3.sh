#!/usr/bin/env bash
# =============================================================================
# SITL 全任务 M3 仿真判分闭环：cuadc_mission 状态机（正式模式）+ 虚拟判定节点
#   + 场景真值感知替身 → SEARCH 真锁定 3 筒 → ALIGN/RELEASE×2（干跑舵机）
#   → /drop_controller/release 判 A/B 区 → RECON 6 航点 → RETURN → LAND → DONE
# 前置：reset_sim.sh 已起 SITL+无头GZ；要看画面另开终端挂 gz sim -g（DISPLAY=:0）
# 门禁：5760 被占即 ABORT（SERIAL0 单客户端）+ FCU 心跳稳定窗（fcu_ready.py）
# 用法：bash ~/sim_scripts/fc_sitl_m3.sh [跟踪秒数,默认 420]
# 验收：投放判定 2/2（争取 A 区）+ 任务总时长 ≤180s（M4 线）
# 注意：sim_release_bridge 仅仿真判分用；真机/正常演练用 fc_sitl_mission.sh
# =============================================================================
set -o pipefail
DUR="${1:-420}"
LOG=/tmp/cuadc_sitl_m3_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-tcp://127.0.0.1:5760}"

echo "[1] 启动 mavros（$FCU）"
# SERIAL0(5760) 单客户端门禁（同 fc_sitl_mission.sh）
if ss -tnp 2>/dev/null | grep -q ':5760'; then
  echo "  ABORT: 5760 已被占用（SERIAL0 单客户端）："
  ss -tnp 2>/dev/null | grep ':5760' | head -3
  exit 1
fi
ros2 launch mavros apm.launch fcu_url:="$FCU" > "$LOG/mavros.log" 2>&1 &
MPID=$!
sleep 5

echo "[2] FCU 就绪门禁（跳过 reset 后不稳定窗）"
if ! python3 "$HOME/sim_scripts/fcu_ready.py" \
     --window "${FCU_STAB_WINDOW:-30}" --timeout "${FCU_READY_TIMEOUT:-600}"; then
  echo "  ABORT: FCU 未就绪。栈活着等自愈即可，反复 reset 会重新进不稳定窗"
  kill "$MPID" 2>/dev/null
  sleep 1
  pkill -f "mavros_node" 2>/dev/null
  exit 1
fi

echo "[3] 消息流订阅（SET_MESSAGE_INTERVAL）"
for mid in 1 30 32 33 74 147 245; do
  timeout 8 ros2 service call /mavros/set_message_interval \
    mavros_msgs/srv/MessageInterval "{message_id: $mid, message_rate: 10.0}" \
    > /dev/null 2>&1
done
echo "  done"

echo "[4] 启动虚拟判定节点 + 场景真值感知替身（真值来自 generated_scene.yaml，带看门狗重启）"
( for i in 1 2 3 4 5 6 7 8 9 10; do
    ros2 run cuadc_rescue_sim virtual_drop_judge_node >> "$LOG/judge.log" 2>&1
    echo "[watchdog] judge 退出(第${i}次), 1s 后重启" >> "$LOG/judge.log"
    sleep 1
  done ) &
JPID=$!
( for i in 1 2 3 4 5 6 7 8 9 10; do
    ros2 run cuadc_perception scene_truth_perception_node >> "$LOG/truth_perception.log" 2>&1
    echo "[watchdog] truth_perception 退出(第${i}次), 1s 后重启" >> "$LOG/truth_perception.log"
    sleep 1
  done ) &
TPID=$!
sleep 2

echo "[5] 状态轨迹记录 + 启动任务节点（正式模式 + 仿真判分桥 + 自动解锁）"
timeout $((DUR + 60)) ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/mission_states.txt" &
STPID=$!
sleep 2
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=false -p sim_release_bridge:=true \
  -p auto_arm_on_guided:=true > "$LOG/mission.log" 2>&1 &
NPID=$!

echo "[6] 外部切 GUIDED（WAIT_GUIDED 放行扳机，等效飞手确认）"
sleep 5
timeout 10 ros2 service call /mavros/set_mode \
  mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" > /dev/null 2>&1 && echo "  GUIDED 已请求"

echo "[7] 跟踪任务（每 15s 快照，DONE/节点退出即收尾）"
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

echo "[8] 收尾清理"
kill "$NPID" 2>/dev/null; sleep 1
pkill -f "cuadc_mission_[n]ode" 2>/dev/null || true
kill "$TPID" "$JPID" "$STPID" 2>/dev/null
pkill -f "scene_truth_perception" 2>/dev/null || true
pkill -f "virtual_drop_judge" 2>/dev/null || true
kill "$MPID" 2>/dev/null; pkill -f mavros_node 2>/dev/null
sleep 2

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹（去重计数）:"
grep -oE '"[A-Z_]+"' "$LOG/mission_states.txt" 2>/dev/null | uniq -c
echo "--- 仿真投放判分（验收：2/2，A 区）:"
grep -ac "release=" "$LOG/judge.log" 2>/dev/null
grep -a "累计=" "$LOG/judge.log" 2>/dev/null
grep -a "仿真投放判定" "$LOG/mission.log" 2>/dev/null
echo "--- mission.log 关键行:"
grep -aE "STATE ->|任务|失败|ABORT|目标集冻结|瞄准点冻结|释放门控|弃桶|arm|ARM|LAND|DONE" \
  "$LOG/mission.log" 2>/dev/null | tail -30
echo "--- mission.log 尾部:"
tail -8 "$LOG/mission.log" 2>/dev/null
