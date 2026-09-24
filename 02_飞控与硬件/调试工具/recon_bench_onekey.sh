#!/usr/bin/env bash
# =============================================================================
# recon_bench_onekey.sh — 判读基准验证一键脚本（09-25 新增，05 册 §6.1 的执行件）
# 一键完成：环境 → 相机 → 单实例 → bench 实例 → 连发递增请求 → 计时收结果
# 背景：判读是事件驱动的（一条 capture_request=一个 4s 窗=一条判读），
#       且 seq 去重（同号忽略）——手动玩经常"迟迟不出结果"，本脚本全自动化。
# 用法（Jetson 上直接跑，或 Windows 侧 ssh jetson 'bash ~/recon_bench_onekey.sh 3'）:
#   bash recon_bench_onekey.sh            # 默认 3 窗
#   bash recon_bench_onekey.sh 5          # 5 窗
#   bash recon_bench_onekey.sh 3 views    # 同时拉起 camera_view/detect_view 观景窗
#   bash recon_bench_onekey.sh stop       # 收尾：停泵/杀 bench/杀查看器/恢复 systemd
# seq 状态存 /tmp/recon_bench_onekey.seq（跨运行递增——同号请求会被去重忽略）
# =============================================================================
set -o pipefail
WS=/home/nvidia/cuadc_ws
PROFILE=/home/nvidia/cuadc_udp_profile.xml
SEQ_FILE=/tmp/recon_bench_onekey.seq
MARKER=/tmp/recon_bench_onekey.stopped_systemd

# ---- 环境三连（照 05 册 §6.1③，缺一不可）----
source /opt/ros/humble/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
[ -f $PROFILE ] && export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE

# ---- 参数解析 ----
N=3; VIEWS=0
case "${1:-}" in
  stop)
    pkill -f "topic pub.*capture_request" 2>/dev/null && echo "已停请求泵" || echo "无请求泵"
    pkill -f hazard_recon_bench 2>/dev/null && echo "已停 bench 实例" || echo "无 bench 实例"
    pkill -f camera_view.py 2>/dev/null && echo "已关 camera_view"
    pkill -f detect_view.py 2>/dev/null && echo "已关 detect_view"
    if [ -f $MARKER ]; then
      sudo -n systemctl start cuadc-perception 2>/dev/null && echo "已恢复 systemd 生产件" \
        || echo "请手动恢复生产件：sudo systemctl start cuadc-perception"
      rm -f $MARKER
    fi
    exit 0 ;;
  *[!0-9]*) N=3 ;;
  *) N="${1:-3}" ;;
esac
[ "${2:-}" = "views" ] && VIEWS=1
[ "$N" -lt 1 ] && N=1

echo "=== [1] 前置检查 ==="
[ -e /dev/video0 ] && echo "  /dev/video0 在位" || { echo "  /dev/video0 缺失——查相机 USB"; exit 1; }

echo "=== [2] 单实例纪律 ==="
if systemctl is-active cuadc-perception >/dev/null 2>&1; then
  if sudo -n systemctl stop cuadc-perception 2>/dev/null; then
    touch $MARKER; echo "  已临时停 systemd 生产件（stop 模式会自动恢复）"
  else
    echo "  ⚠️ systemd 生产件仍在跑且 sudo 需密码——结果会双份，建议先手动：sudo systemctl stop cuadc-perception"
  fi
else
  echo "  systemd 生产件未运行（bench 单实例，OK）"
fi

echo "=== [3] 相机 ==="
if ! ros2 topic info /camera/image_raw/compressed 2>/dev/null | grep -q "Publisher count: [1-9]"; then
  if pgrep -f "camera_node[.]py" >/dev/null; then
    echo "  camera_node 进程在但不出流——杀掉重起"; pkill -f "camera_node[.]py"; sleep 2
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE
  nohup python3 /home/nvidia/camera_node.py > /tmp/camera_node.log 2>&1 < /dev/null &
  sleep 5 && tail -1 /tmp/camera_node.log
fi
pc=$(timeout 6 ros2 topic info /camera/image_raw/compressed 2>/dev/null | grep -o "Publisher count: [0-9]*" | grep -o "[0-9]*")
[ "${pc:-0}" -ge 1 ] && echo "  相机出流 OK（publisher=$pc）" || { echo "  ❌ 相机无流——查 05 册 §6.1①"; exit 1; }

echo "=== [4] bench 判读实例 ==="
if ros2 node list 2>/dev/null | grep -q hazard_recon_bench; then
  echo "  bench 实例已在跑，复用"
else
  export PYTHONPATH=$HOME/cuadc_ws/src/cuadc_perception:$PYTHONPATH
  nohup python3 $WS/src/cuadc_perception/cuadc_perception/hazard_recon_node.py \
    --ros-args -r __node:=hazard_recon_bench \
    -p image_compressed:=true -p image_qos_reliable:=false \
    -p require_recon_state:=false > /tmp/recon_bench.log 2>&1 < /dev/null &
  sleep 8
  grep -q "真判读就绪" /tmp/recon_bench.log && echo "  bench 就绪" || { echo "  ❌ bench 未就绪，看 /tmp/recon_bench.log"; exit 1; }
fi

if [ "$VIEWS" = "1" ]; then
  echo "=== [4b] 拉起查看器 ==="
  DISPLAY=:0 nohup python3 /home/nvidia/camera_view.py > /tmp/camera_view.log 2>&1 < /dev/null &
  DISPLAY=:0 nohup python3 /home/nvidia/detect_view.py > /tmp/detect_view.log 2>&1 < /dev/null &
  sleep 2; echo "  两窗已拉（同位叠放，拖开；按 q 退）"
fi

echo "=== [5] 连发 $N 窗并计时收结果 ==="
last=$(cat $SEQ_FILE 2>/dev/null || echo 100)
for i in $(seq 1 $N); do
  seq=$((last + i))
  t0=$(date +%s.%N)
  timeout 3.5 ros2 topic pub -r 1 /cuadc/recon/capture_request \
    geometry_msgs/msg/PointStamped "{header: {frame_id: ''}, point: {x: ${seq}.0, y: 0, z: 0}}" >/dev/null 2>&1 &
  PUB=$!
  # 等本窗判读到达（viewpoint_seq 匹配本窗序号；9s 超时）。
  # ⚠️ 管道必须关进 bash -c 子壳：脚本顶部 pipefail 会让上游 SIGPIPE(141) 盖过 grep 匹配的 0，
  #    外层直连管道会把"匹配成功"误判成失败（09-25 实测）。
  if timeout 9 bash -c "ros2 topic echo /cuadc/recon/classification 2>/dev/null | grep -m1 -E 'viewpoint_seq: *${seq} *\$' >/dev/null" 2>/dev/null; then
    arr=$(date +%s.%N)
  else
    arr=""
  fi
  wait $PUB 2>/dev/null
  ev=$(ls -t /home/nvidia/recon_evidence 2>/dev/null | head -1)
  if [ -n "$arr" ]; then
    lat=$(python3 -c "print(f'{$arr-$t0:.2f}')")
    echo "  窗 $seq/$N: ✅ 判读到达 ${lat}s | 证据: $ev"
    cat /home/nvidia/recon_evidence/$ev/*.jsonl 2>/dev/null | tail -1 | head -c 260; echo
  else
    echo "  窗 $seq/$N: ❌ 9s 未见到判读（相机帧/实例/摆靶自查，见 05 册 §6.1⑦）"
  fi
done
echo $seq > $SEQ_FILE
echo "=== 完成：seq 已推进到 $seq（记录于 $SEQ_FILE），证据在 ~/recon_evidence ==="
echo "收尾请跑: bash $(basename $0) stop"
