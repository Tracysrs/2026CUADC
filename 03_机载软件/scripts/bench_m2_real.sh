#!/usr/bin/env bash
# =============================================================================
# M2 真机台架联测 · 分阶段脚本（FC TELEM3 /dev/ttyTHS1:921600）
#
# 用法：bash bench_m2_real.sh <stage>
#   camera     相机单实例拉起 + 30Hz 流校验（无 FC 依赖，安全）
#   mavros     MAVROS 真链拉起 + fcu_ready 门禁（需 FC 已上电）
#   perception bucket_perception 节点拉起（需 camera；SHA 过→主通道 seg+LAB）
#   check      逐级验收：流 Hz / 节点表 / 感知输出 PoseArray / 双通道日志
#   mission    任务节点（sim_release_bridge:=true 干跑舵机）+ 外部 GUIDED
#   stop       全部收尾
#
# 物理前置（人）：FC 上电（TELEM3 线 = 8/10/6 三线）、相机 USB、真瓶上架、
#               拆桨确认、开灯（bench 方案 brightness=48+开灯）。
# 纪律：mission 阶段 props 必须已拆；release 实弹第二遍才 sim_release_bridge:=false。
# =============================================================================
set -o pipefail
S="${1:?用法: bench_m2_real.sh camera|mavros|perception|check|mission|stop}"
LOG=/tmp/cuadc_bench_m2
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU_URL="/dev/ttyTHS1:921600"

case "$S" in
camera)
  pgrep -f "camera_node[.]py" | xargs -r kill; sleep 2
  nohup python3 "$HOME/camera_node.py" > "$LOG/camera.log" 2>&1 < /dev/null &
  sleep 3
  echo "--- 流校验（5s 采样）："
  timeout 6 ros2 topic hz /camera/image_raw/compressed 2>/dev/null | head -2
  pgrep -f "camera_node[.]py" | wc -l | xargs echo "camera 实例数（应为 1）:"
  ;;
mavros)
  pkill -f "mavros_[n]ode" 2>/dev/null; sleep 1
  ros2 launch mavros apm.launch fcu_url:="$FCU_URL" > "$LOG/mavros.log" 2>&1 &
  sleep 5
  # 真机就绪判据 = /mavros/state connected:true（fcu_ready.py 的 TCP 探测只适用 SITL，勿用）
  CONN=""
  for i in $(seq 1 24); do
    CONN=$(timeout 4 ros2 topic echo --once /mavros/state 2>/dev/null | grep -m1 'connected: true')
    [ -n "$CONN" ] && { echo "FCU 心跳 OK（第 ${i} 探）"; break; }
    sleep 5
  done
  [ -n "$CONN" ] || { echo '!! 2 分钟未收到 FC 心跳：查 FC 上电、TELEM3 三线（8/10/6）、波特率 921600'; exit 1; }
  for mid in 1 30 32 33 74 147 245; do
    timeout 8 ros2 service call /mavros/set_message_interval \
      mavros_msgs/srv/MessageInterval "{message_id: $mid, message_rate: 10.0}" \
      > /dev/null 2>&1
  done
  echo "MAVROS 就绪 + 消息流已配"
  ;;
perception)
  pkill -f "bucket_perception_[n]ode" 2>/dev/null; sleep 1
  ros2 launch cuadc_perception bucket_perception.launch.py \
    > "$LOG/bucket_perception.log" 2>&1 &
  sleep 6
  grep -aE "主通道|SHA|降级|LAB|engine" "$LOG/bucket_perception.log" | head -8
  ;;
check)
  echo "== 相机 =="
  timeout 5 ros2 topic hz /camera/image_raw/compressed 2>/dev/null | tail -1
  echo "== 节点 =="
  ros2 node list 2>/dev/null | grep -E "bucket|mission|mavros|camera" | sort
  echo "== odom（MAVROS→FC）=="
  timeout 4 ros2 topic echo --once /mavros/local_position/odom 2>/dev/null | grep -m1 -A2 "pose" | head -3
  echo "== 感知输出（桶放镜头下应有 PoseArray）=="
  timeout 6 ros2 topic echo --once /perception/drop_buckets_body 2>/dev/null | head -6
  echo "== 感知日志关键行 =="
  grep -aE "主通道|双通道|降级|SHA|检出" "$LOG/bucket_perception.log" 2>/dev/null | tail -6
  ;;
mission)
  # 干跑：sim_release_bridge:=true（舵机指令只打印不落地）；实弹第二遍改 false
  pkill -f "cuadc_mission_[n]ode" 2>/dev/null; sleep 1
  DRY="${2:-true}"
  echo "任务节点启动（sim_release_bridge:=$DRY）——状态轨迹 60s："
  ( for i in 1 2 3; do
      ros2 run cuadc_mission cuadc_mission_node --ros-args \
        --params-file "$HOME/cuadc_ws/src/cuadc_mission/config/mission_params.yaml" \
        -p m1_no_vision_mode:=false -p sim_release_bridge:=$DRY \
        -p active_states:="SEARCH,ALIGN" \
        >> "$LOG/mission.log" 2>&1
      sleep 1
    done ) &
  sleep 8
  timeout 12 ros2 service call /mavros/set_mode \
    mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" > /dev/null 2>&1 && echo "GUIDED 已请求"
  echo "--- 状态轨迹（干跑观察 60s，Ctrl+C 停）:"
  timeout 60 ros2 topic echo /cuadc/mission_state 2>/dev/null | grep --line-buffered "data:"
  echo "--- mission 关键行:"
  grep -aE "STATE ->|目标|释放|DROP|ABORT" "$LOG/mission.log" 2>/dev/null | tail -12
  ;;
stop)
  pkill -f "cuadc_mission_[n]ode" 2>/dev/null
  pkill -f "bucket_perception_[n]ode" 2>/dev/null
  pkill -f "mavros_[n]ode" 2>/dev/null
  pkill -f "camera_node[.]py" 2>/dev/null
  echo "已全部收尾"
  ;;
*) echo "未知 stage: $S"; exit 1 ;;
esac
