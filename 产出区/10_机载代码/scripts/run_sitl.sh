#!/usr/bin/env bash
# =============================================================================
# 一键拉起 SITL 全链路：Gazebo 场景 → ArduPilot SITL → MAVROS → 任务状态机
#
# 用法：
#   ./run_sitl.sh                 # 标准四件套
#   ./run_sitl.sh --regen         # 先重新生成随机比赛场景再起（SETUP.md §5）
#   FCU_URL=udp://:14550@ ./run_sitl.sh   # 覆盖 MAVLink 端点
#
# 停止：Ctrl+C（trap 自动清理 Gazebo / SITL / MAVROS / 任务节点）
# 日志：/tmp/cuadc_sitl_logs/
#
# 注意：HIT 仿真包默认不开 ros_gz_bridge，没有 /clock 话题，
#       所以任务节点固定用系统时钟（use_sim_time:=false）。SITL 实时运行无影响。
# =============================================================================
set -uo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
WORKSPACE="${WORKSPACE:-$HOME/cuadc_ws}"
SIM_PKG="${SIM_PKG:-cuadc_rescue_sim}"
FCU_URL="${FCU_URL:-udp://:14550@}"
LOGDIR=/tmp/cuadc_sitl_logs
PIDS=()

log()    { echo "[run_sitl] $*"; }
die()    { echo -e "\033[31m[错误]\033[0m $*" >&2; exit 1; }

cleanup() {
  echo ""
  log "清理进程 …"
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null; done
  pkill -f sim_vehicle.py 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
}
trap cleanup EXIT INT TERM

start() {  # start <名字> <命令...>   后台启动并记 PID/日志
  local name="$1"; shift
  log "启动 $name（日志: $LOGDIR/$name.log）"
  "$@" >"$LOGDIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

# ---- 预检 ----
mkdir -p "$LOGDIR"
[ -d "$ARDUPILOT_DIR/Tools/autotest" ] || die "找不到 $ARDUPILOT_DIR —— 先跑 setup_env_ubuntu22.sh"
[ -f "$WORKSPACE/install/setup.bash" ] || die "工作空间未构建 —— 先跑 setup_env_ubuntu22.sh"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"

# ---- 可选：重新生成随机场景 ----
if [[ "${1:-}" == "--regen" ]]; then
  log "重新生成随机比赛场景 …"
  (cd "$WORKSPACE/src/$SIM_PKG" && python3 scripts/generate_scene.py) || die "场景生成失败"
  (cd "$WORKSPACE" && colcon build --packages-select "$SIM_PKG") || die "重新构建失败"
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
fi

# ---- 1) Gazebo 场景 ----
start gz_sim ros2 launch "$SIM_PKG" cuadc_sim.launch.py
sleep 8

# ---- 2) ArduPilot SITL（gazebo-iris + JSON 对接插件，14550 推流给 MAVROS）----
start sitl python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter -f gazebo-iris --model JSON \
  --out=udp:127.0.0.1:14550 --no-map --no-console
sleep 15

# ---- 3) MAVROS ----
start mavros ros2 launch mavros apm.launch fcu_url:="$FCU_URL"

# ---- 等飞控连接（最多约 30s）----
log "等待 MAVROS 连接飞控 …"
connected=0
for _ in $(seq 1 15); do
  if timeout 3 ros2 topic echo --once /mavros/state --field connected 2>/dev/null | grep -q true; then
    connected=1
    break
  fi
  sleep 2
done
[ "$connected" = "1" ] || log "警告：30s 内未确认连接（继续启动任务节点，日志见 $LOGDIR）"

# ---- 4) 任务状态机 ----
start mission ros2 launch cuadc_mission mission.launch.py

echo ""
echo "=============================================================="
echo " SITL 全链路已拉起。观察："
echo "   ros2 topic echo /cuadc/mission_state     # 状态轨迹"
echo "   tail -f $LOGDIR/mission.log              # 任务节点日志"
echo "   tail -f $LOGDIR/gz_sim.log               # 仿真日志"
echo " 停止：Ctrl+C（自动清理全部进程）"
echo "=============================================================="

# 前台等任务节点退出；Ctrl+C 触发 trap 清理
wait "${PIDS[-1]}"
