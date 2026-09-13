#!/usr/bin/env bash
# =============================================================================
# SITL 全任务 M2 视觉在环验收：cuadc_mission 状态机（正式模式）+ 虚拟判定节点
#   + **CV 白桶真感知**（gz 相机 → LAB 分割 → 单目解算 → 契约 PoseArray）
#   → SEARCH 真锁定 3 筒 → ALIGN/RELEASE×2（干跑舵机）→ 判 A/B 区 → 侦察 → 降落
# 与 fc_sitl_m3.sh 的差别：感知源 = bucket_cv_perception_node（gz-transport 直订
#   /d435i/image，无需 ros_gz 桥），不是场景真值替身——检测/直径/置信度全部
#   来自真图像管线，M3 的真值替身保留作对照基准。
# 前置：gz server 必须带 Sensors 渲染（世界文件已加，2026-09-12）；DISPLAY=:0。
# 门禁：5760 被占即 ABORT + FCU 心跳稳定窗（fcu_ready.py）
# 用法：bash ~/sim_scripts/fc_sitl_m2.sh [跟踪秒数,默认 420]
# =============================================================================
set -o pipefail
DUR="${1:-420}"
LOG=/tmp/cuadc_sitl_m2_$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/cuadc_ws/install/setup.bash"
FCU="${FCU_URL:-tcp://127.0.0.1:5760}"

echo "[0] 强清残留（僵尸任务节点会共享 mavros 抢指令——2026-09-13 实证：
#    残留 DISARM 态节点每秒发 disarm 把新任务在空中打下来）"
pkill -f "cuadc_mission_[n]ode" 2>/dev/null
pkill -f "mavros_[n]ode" 2>/dev/null
pkill -f "bucket_cv_[p]erception" 2>/dev/null
pkill -f "virtual_drop_[j]udge" 2>/dev/null
pkill -f "scene_truth_[p]erception" 2>/dev/null
sleep 2

echo "[1] 启动 mavros（$FCU）"
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

echo "[4] 启动虚拟判定节点 + CV 白桶真感知（gz 相机直订，带看门狗重启）"
# 判定节点桶真值随 randomize_scene.py 布景联动（无此文件=旧默认桶位）
JP=""
[ -f "$HOME/sim_scripts/judge_truth_params.yaml" ] && \
  JP="--ros-args --params-file $HOME/sim_scripts/judge_truth_params.yaml"
echo "  judge 桶真值: ${JP:+judge_truth_params.yaml（随机布景）}${JP:-默认（旧固定桶位）}"
( for i in 1 2 3 4 5 6 7 8 9 10; do
    ros2 run cuadc_rescue_sim virtual_drop_judge_node $JP >> "$LOG/judge.log" 2>&1
    echo "[watchdog] judge 退出(第${i}次), 1s 后重启" >> "$LOG/judge.log"
    sleep 1
  done ) &
JPID=$!
( for i in 1 2 3 4 5 6 7 8 9 10; do
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
      ros2 run cuadc_perception bucket_cv_perception_node --ros-args \
      -p l_min:=150.0 -p debug_save_period:=120 \
      >> "$LOG/cv_perception.log" 2>&1
    echo "[watchdog] cv_perception 退出(第${i}次), 1s 后重启" >> "$LOG/cv_perception.log"
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
  -p auto_arm_on_guided:=true \
  -p search_speed_m_s:=1.2 -p search_degrade_timeout_s:=120.0 \
  > "$LOG/mission.log" 2>&1 &
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
pkill -f "bucket_cv_[p]erception" 2>/dev/null || true
pkill -f "virtual_drop_judge" 2>/dev/null || true
kill "$MPID" 2>/dev/null; pkill -f mavros_node 2>/dev/null
sleep 2

echo "=== 完成，日志目录: $LOG ==="
echo "--- 状态轨迹（去重计数）:"
grep -oE '"[A-Z_]+"' "$LOG/mission_states.txt" 2>/dev/null | uniq -c
echo "--- CV 感知关键行:"
grep -aE "帧 |检出|异常|ERROR" "$LOG/cv_perception.log" 2>/dev/null | tail -8
echo "--- 仿真投放判分（验收：2/2，A 区）:"
grep -a "累计=" "$LOG/judge.log" 2>/dev/null
grep -a "仿真投放判定" "$LOG/mission.log" 2>/dev/null
echo "--- mission.log 关键行:"
grep -aE "STATE ->|任务|失败|ABORT|目标集冻结|瞄准点冻结|释放门控|弃桶" \
  "$LOG/mission.log" 2>/dev/null | tail -25
echo "--- mission.log 尾部:"
tail -8 "$LOG/mission.log" 2>/dev/null

echo ""
echo "========== 完成事项 =========="
python3 - "$LOG/mission.log" "$LOG/judge.log" << 'PYEOF'
import re, sys

mission = open(sys.argv[1], encoding='utf-8', errors='replace').read()
try:
    judge = open(sys.argv[2], encoding='utf-8', errors='replace').read()
except OSError:
    judge = ''

states = re.findall(r'\[(?:INFO|ERROR)\] \[(\d+)\.\d+\] \[cuadc_mission\]: STATE -> (\w+)', mission)
first = {}
for ts, st in states:
    first.setdefault(st, int(ts))
t0 = first.get('WAIT_NAV_STABLE')
done = first.get('DONE')

items, total = [], None
if t0 and done:
    total = done - t0
    def dur(a, b):
        if a in first and b in first:
            return '%.0fs' % (first[b] - first[a])
        return '?'
    items.append('✅ 起飞（%s）' % dur('WAIT_ARM', 'SEARCH')
                 if 'SEARCH' in first else '✅ 起飞')
lock = re.search(r'目标集冻结\((\w+), (\d+) 筒\)', mission)
if lock:
    items.append('✅ 搜索锁定（%s 筒, %s，搜索段 %s）'
                 % (lock.group(2), lock.group(1), dur('SEARCH', 'ALIGN')))
rel = re.findall(
    r'success=(\d) release=(\d) target=(\S+).*?error=([\d.]+) zone=(\w+) score=(\d+)',
    mission)
for _s, n, tgt, err, zone, sc in rel:
    mark = '✅' if zone == 'A' else ('⚠️' if zone == 'B' else '❌')
    items.append('%s 投放#%s → %s（误差 %.1fcm，%s 区 %s 分）'
                 % (mark, n, tgt, float(err) * 100, zone, sc))
photos = re.search(r'任务结束: 投放 (\d)/2, 拍照确认数=(\d+)', mission)
if photos:
    items.append('📸 侦察拍照确认 %s/6' % photos.group(2))
if 'DISARM' in first or 'DONE' in first:
    items.append('✅ 返航降落上锁')
score = re.findall(r'累计=(\d+)', judge)
if 'PILOT_OVERRIDE' in first:
    items.append('⚠️ 飞手接管退出')
reason = re.search(r'失败原因: (.+)$', mission, re.M)
if reason:
    items.append('❌ 未完成: ' + reason.group(1))

print(' | '.join(items) if items else '任务未产生有效记录')
if total is not None:
    print('总时长: %.0f s（验收线 180s / 硬上限 240s）' % total)
if score:
    print('投放判分累计: %s 分（conservative 序满分 400=100+300；aggressive 序冲 800=500+300）' % score[-1])
PYEOF
