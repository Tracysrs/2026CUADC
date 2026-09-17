#!/usr/bin/env bash
# =============================================================================
# 任务模式统一入口：一个命令选模式
# 用法：bash ~/sim_scripts/fc_task.sh [mission|m3|m2] [跟踪秒数]
#   mission (m1) : M1 无视觉演练——纯预设航线，回归飞行链路
#   m3      (默认): M3 判分闭环——真值替身感知，投放判分（随机布景联动 judge）
#   m2      : M2 真视觉——CV 相机检测（gz 渲染世界 + 相机出流才可用）
# 前置流程（见 操作手册.md）：reset_sim.sh → fcu_ready.py → 本脚本
# =============================================================================
MODE="${1:-m3}"
DUR="${2:-}"

case "$MODE" in
  mission|m1)
    echo "=== 任务模式: M1 无视觉演练 ==="
    exec bash ~/sim_scripts/fc_sitl_mission.sh ${DUR:+$DUR}
    ;;
  m3|truth)
    echo "=== 任务模式: M3 判分闭环（真值替身感知）==="
    exec bash ~/sim_scripts/fc_sitl_m3.sh ${DUR:+$DUR}
    ;;
  m2|cv)
    echo "=== 任务模式: M2 真视觉（CV 相机感知）==="
    exec bash ~/sim_scripts/fc_sitl_m2.sh ${DUR:+$DUR}
    ;;
  *)
    echo "用法: bash $0 [mission|m3|m2] [跟踪秒数]"
    echo "  mission (m1): M1 无视觉演练"
    echo "  m3     (默认): M3 判分闭环（真值替身）"
    echo "  m2     : M2 真视觉（CV 相机）"
    exit 1
    ;;
esac
