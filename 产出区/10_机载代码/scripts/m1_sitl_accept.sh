#!/usr/bin/env bash
# =============================================================================
# M1 里程碑验收 · ArduPilot SITL（无 Gazebo 轻量链路）+ MAVROS + 任务状态机
#
# 用法（在 Jetson 上）：
#   bash m1_sitl_accept.sh flow       # 验收A：全流程无人工干预到 DONE
#   bash m1_sitl_accept.sh override   # 验收B：切出 GUIDED → PILOT_OVERRIDE 接管保护
#   bash m1_sitl_accept.sh odomstale  # 验收C：冻结 SITL 模拟断 odom → 失败保护
#
# 依据：10_机载代码/README.md M1 验收清单 + SSOT §7（每版先过 SITL 再上真机）
# 说明：不走 run_sitl.sh 的 Gazebo 路线——M1 验收只需 FCU 仿真 + MAVROS +
#       状态机；视觉由 m1_no_vision_mode 旁路。日志在 /tmp/m1_<pid>/
# =============================================================================
# set -u 不可用: ROS 2 setup.bash 引用未绑定变量(AMENT_TRACE_SETUP_FILES)
MODE="${1:-flow}"
ARDU_DIR="$HOME/ardupilot"
WS="$HOME/cuadc_ws"
LOG="/tmp/m1_$$"
mkdir -p "$LOG"
SPID=""; MPID=""; NPID=""; CPID=""; PUMP=""

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  for p in "$SPID" "$MPID" "$NPID" "$CPID" "$PUMP"; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  pkill -f "build/sitl/bin/arducopter" 2>/dev/null
  pkill -f "mavros_node" 2>/dev/null
}
trap cleanup EXIT

set_guided() {        # $1=模式名;CLI 的 DDS 发现偶发挂死 -> 15s 超时 + 3 次重试
  local i
  for i in 1 2 3; do
    timeout 15 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: '$1'}" > /dev/null 2>&1 && return 0
  done
  echo "WARN: set_mode $1 三次尝试未确认(继续)"
  return 1
}

wait_for_state() {   # $1=状态名 $2=超时秒
  local i
  for i in $(seq 1 "$2"); do
    grep -q "STATE -> $1" "$LOG/mission.log" 2>/dev/null && return 0
    sleep 1
  done
  return 1
}

echo "[1] 启动 SITL arducopter（-w 恢复默认参数，home=Perth 演示值）"
cd "$ARDU_DIR"
./build/sitl/bin/arducopter -w --model quad --defaults $HOME/cuadc_sitl_defaults.parm --home=-31.95,115.85,15,0 > "$LOG/sitl.log" 2>&1 &
SPID=$!

echo "[2] 启动 MAVROS（tcp://127.0.0.1:5760）"
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 gcs_url:=tcp-l://0.0.0.0:14550 > "$LOG/mavros.log" 2>&1 &
MPID=$!
# GCS 心跳泵:ArduPilot 只有在端口上看到 GCS 心跳才开始流送(mavros 不冒充 GCS)
sleep 2
nohup python3 $HOME/gcs_pump.py > "$LOG/pump.log" 2>&1 &
PUMP=$!
MPID=$!

echo "[3] 等 SITL 启动 + EKF/GPS 收敛（20s）"
sleep 20

echo "[4] 状态记录 + 任务节点（m1 无视觉演练 + GUIDED 后自动解锁）"
timeout 240 ros2 topic echo /cuadc/mission_state 2>/dev/null \
  | grep --line-buffered "data:" > "$LOG/states.txt" &
CPID=$!
ros2 run cuadc_mission cuadc_mission_node --ros-args \
  --params-file "$WS/src/cuadc_mission/config/mission_params.yaml" \
  -p m1_no_vision_mode:=true -p auto_arm_on_guided:=true \
  > "$LOG/mission.log" 2>&1 &
NPID=$!
sleep 5

case "$MODE" in
  flow)
    echo "[5] 切 GUIDED（最终确认，任务自动解锁起飞）"
    set_guided GUIDED
    if wait_for_state DONE 220 || grep -q "任务结束" "$LOG/mission.log"; then
      echo "RUN-A PASS"
      echo "状态轨迹: $(grep "STATE -> " "$LOG/mission.log" | awk '{print $NF}' | tr '\n' ' ')"
      grep -E "任务结束" "$LOG/mission.log" | tail -1
      exit 0
    fi
    echo "RUN-A FAIL：220s 未到 DONE"
    echo "--- 状态轨迹(部分): $(grep "STATE -> " "$LOG/mission.log" | awk '{print $NF}' | tr '\n' ' ')"
    tail -8 "$LOG/mission.log"
    exit 1
    ;;
  override)
    echo "[5] 切 GUIDED"
    set_guided GUIDED
    echo "[6] 等进入 TAKEOFF..."
    if ! wait_for_state TAKEOFF 60; then
      echo "RUN-B FAIL：未进入 TAKEOFF"; tail -5 "$LOG/mission.log"; exit 1
    fi
    sleep 2
    echo "[7] 模拟飞手接管：切 ALT_HOLD"
    set_guided ALT_HOLD
    sleep 4
    if grep -q "PILOT_OVERRIDE" "$LOG/mission.log" && \
       grep -q "STATE -> PILOT_OVERRIDE" "$LOG/mission.log"; then
      echo "RUN-B PASS：接管保护触发，任务立即停发目标点退让（绝不与飞手抢权）"
      exit 0
    fi
    echo "RUN-B FAIL"; tail -5 "$LOG/mission.log"; exit 1
    ;;
  odomstale)
    set_guided GUIDED
    if ! wait_for_state TAKEOFF 60; then
      echo "RUN-C FAIL：未进入 TAKEOFF"; tail -5 "$LOG/mission.log"; exit 1
    fi
    sleep 3
    echo "[7] SIGSTOP 冻结 SITL（模拟 odom 断流，MAVROS 存活）"
    kill -STOP $SPID
    sleep 8
    if grep -q -E "MISSION_(ABORT|FAILURE)" "$LOG/mission.log"; then
      echo "RUN-C PASS：断定位保护触发 → $(grep -E 'MISSION_(ABORT|FAILURE)' "$LOG/mission.log" | head -1)"
      kill -CONT $SPID 2>/dev/null
      exit 0
    fi
    echo "RUN-C FAIL：8s 内未见失败路径"; tail -5 "$LOG/mission.log"
    kill -CONT $SPID 2>/dev/null
    exit 1
    ;;
  *)
    echo "用法: bash $0 flow|override|odomstale"; exit 1
    ;;
esac
