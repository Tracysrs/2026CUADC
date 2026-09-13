#!/usr/bin/env bash
# CUADC 感知自启入口（systemd 拉起，P0.3）
# 职责：ROS2 环境 → 工作空间 → 起真侦察判读节点（加载 engine + 黑帧预热后待命）
# 注意：engine 缺失/SHA 不符时节点拒启，systemd 会按 Restart=on-failure 重试——
#       这是故意的 fail-closed（宁可反复失败留痕，不许带错权重上岗）。
# UDP-only Fast DDS profile：/dev/shm 共享内存段被同机其他 DDS 进程搅乱时仍能
# 通信（09-12 实测：与仿真栈同机时 SHM 匹配失败、节点不可见，UDP-only 修复）。
source /opt/ros/humble/setup.bash
source /home/nvidia/cuadc_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/cuadc_udp_profile.xml
exec ros2 run cuadc_perception hazard_recon_node --ros-args \
  --params-file /home/nvidia/cuadc_ws/install/cuadc_perception/share/cuadc_perception/config/hazard_recon_params.yaml
