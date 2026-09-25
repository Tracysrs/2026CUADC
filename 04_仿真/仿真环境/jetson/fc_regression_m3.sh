#!/usr/bin/env bash
# =============================================================================
# fc_regression_m3.sh —— A 批代码上机闸门一键回归（2026-09-20 新增，SSOT §15 排期）
#
# 背景：09-20 两批 mission_node/drop_logic 改动（舵机归因、tick 顺序、odom 历史
# 换型等）本机无 ROS 环境不编译——上机前必须过本回归，全绿才算闸门通过。
#
# 用法：bash ~/sim_scripts/fc_regression_m3.sh
# 前置：Jetson 已连（ssh jetson）；无真飞控/真感知依赖（SITL 全栈本机自足）。
# 流程：
#   [1/4] colcon build（cuadc_ws，失败即中止——编译门禁）
#   [2/4] 单元测试：Python 镜像（unittest discover）+ C++ 三件（drop/route/odom_interp）
#   [3/4] reset_sim.sh 全栈重置（约 40s）
#   [4/4] fc_sitl_m3.sh 全任务判分（最长 420s，结束自动打分）
# 判据（全部满足才 REGRESSION PASS）：
#   colcon 无 error；单测全绿；投放 2/2 全 A 区；总时长 ≤180s
# 修复 2026-09-25 首跑死锁：stage3/4 原用 "| tail/tee" 接输出，但 reset_sim/fc_sitl_m3
#   拉起的守护子进程（SITL 的 sleep infinity 管道树/gz/mavros）继承管道写端永不关闭，
#   tail 等不到 EOF 整脚本卡死（bash -n 查不出的运行时缺陷）——改为输出落文件、结束后 tail。
# =============================================================================
set -o pipefail
LOG=/tmp/cuadc_regression_$(date +%m%d_%H%M%S)
mkdir -p "$LOG" /tmp/cuadc_cpp_tests
FAIL=0

echo "[1/4] colcon build（cuadc_ws）"
if ! (source /opt/ros/humble/setup.bash && cd "$HOME/cuadc_ws" && colcon build 2>&1 | tail -6); then
  echo "REGRESSION FAIL: colcon build 失败（log: $LOG）"
  exit 1
fi

echo "[2/4] 单元测试（Python 镜像 + C++ 三件）"
if (cd "$HOME/cuadc_ws/src/cuadc_mission" \
    && python3 -m unittest discover -s tools 2>&1 | tail -3); then
  :
else
  echo "REGRESSION FAIL: Python 单测未全绿"
  FAIL=1
fi
cd "$HOME/cuadc_ws/src/cuadc_mission/test"
for t in drop_logic route_logic odom_interp; do
  if g++ -std=c++17 -I ../include "test_$t.cpp" -o "/tmp/cuadc_cpp_tests/$t" \
      2>"/tmp/cuadc_cpp_tests/$t.build.log" \
      && "/tmp/cuadc_cpp_tests/$t" | tail -1; then
    :
  else
    echo "  ⚠️ C++ 测试 $t 编译/运行失败（build log: /tmp/cuadc_cpp_tests/$t.build.log）"
    FAIL=1
  fi
done
if [ "$FAIL" -ne 0 ]; then
  echo "REGRESSION FAIL（单测阶段，log: $LOG）"
  exit 1
fi

echo "[3/4] 全栈重置（reset_sim.sh，约 40s；输出落 $LOG/reset_sim.txt）"
if ! bash "$HOME/sim_scripts/reset_sim.sh" >"$LOG/reset_sim.txt" 2>&1; then
  echo "REGRESSION FAIL: reset_sim 失败（$LOG/reset_sim.txt 末 5 行：）"
  tail -5 "$LOG/reset_sim.txt"
  exit 1
fi
tail -3 "$LOG/reset_sim.txt"

echo "[4/4] SITL M3 全任务判分（最长 420s，结束自动打分；输出落 $LOG/m3.txt）"
if ! bash "$HOME/sim_scripts/fc_sitl_m3.sh" 420 >"$LOG/m3.txt" 2>&1; then
  echo "⚠️ fc_sitl_m3 退出码非 0（$LOG/m3.txt 末 15 行：）"
fi
tail -15 "$LOG/m3.txt"

# ---- 自动判定（判据与 fc_sitl_m3 输出口径对齐）----
PASS=1
grep -q "投放#1 → .*A 区" "$LOG/m3.txt" || { echo "❌ 投放#1 未进 A 区"; PASS=0; }
grep -q "投放#2 → .*A 区" "$LOG/m3.txt" || { echo "❌ 投放#2 未进 A 区"; PASS=0; }
TOTAL=$(grep -oP '总时长: \K[0-9]+' "$LOG/m3.txt" | head -1)
if [ -n "$TOTAL" ] && [ "$TOTAL" -le 180 ]; then
  echo "✅ 总时长 ${TOTAL}s ≤180s"
else
  echo "❌ 总时长缺失或 >180s（${TOTAL:-无输出}）"; PASS=0
fi
echo "log: $LOG/m3.txt"
if [ "$PASS" -eq 1 ]; then
  echo "REGRESSION PASS ✅（闸门通过：A 批改动放行）"
  exit 0
fi
echo "REGRESSION FAIL ❌（闸门不通过：修复后重跑本脚本）"
exit 1
