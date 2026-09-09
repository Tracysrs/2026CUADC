# 2026-09-09 真飞控 × Jetson USB 链路（干跑 + 解锁试跑）归档

清单 #6 前半（USB 段）首次打通 + §九解锁试跑的现场记录。执行脚本存档于
`../fc_dryrun.sh`、`../fc_armrun.sh`、`../fc_armtest.sh`，当日日志目录已拷回本目录。

## 结论速览

- mavros 经 `/dev/cuadc-fc:115200` 连真飞控：**起动后 2s 收到心跳**；
  GCS 心跳泵（gcs_pump.py + tcp-l:14550）触发流送后遥测全通
- 电池读数 22.42V / 剩余 33% / -0.06A（CAN PMU 正常；已低于解锁门槛，该充电）
- GUIDED 切换成功（`mode_sent=True`，state 回读 mode=GUIDED / connected=true）
- 任务状态机干跑（无桨，auto_arm=false）：`WAIT_FCU → WAIT_NAV_STABLE` 后停滞
  90s，卡点 = 无 GPS、EKF 无位置（预期内，等加速度计/罗盘校准 + GPS 到货）
- FC 周期广播 PreArm 五项：GPS 1 bad fix / Battery below min arming voltage /
  Radio failsafe on / 3D Accel calibration needed / Compass not calibrated
  ——与待办清单 #1~#4 一一对应
- dataflash 回拉：`python3 -m pymavlink.mavftp --device /dev/cuadc-fc
  --burst_read_size 239 get /APM/Logs/00000012.BIN <本地>`，64MB/约90s（~815KB/s）

## 文件清单

| 文件 | 内容 |
|---|---|
| `cuadc_fc_dryrun_0909_144100/mavros.log` | 连接时序：起动 → 2s 心跳（CON: Got HEARTBEAT）→ PreArm 广播 |
| `cuadc_fc_dryrun_0909_144100/battery.txt` | /mavros/battery 快照：22.42V / 33% / -0.06A |
| `cuadc_fc_dryrun_0909_144100/vfr_hud.txt` | 航向 227°（罗盘未校准乱值）、地速≈0、气压高 0.02m |
| `cuadc_fc_dryrun_0909_144100/state_after_setmode.txt` | GUIDED 切换后：connected=true / armed=false / mode=GUIDED |
| `cuadc_fc_dryrun_0909_144100/setmode.txt` | SetMode 服务应答 mode_sent=True |
| `cuadc_fc_dryrun_0909_144100/statustext.txt` | FC 广播的 PreArm 五项（周期三轮） |
| `cuadc_fc_dryrun_0909_144100/mission.log` | 状态机干跑：ready → WAIT_NAV_STABLE 停滞（M1 无视觉演练模式告警） |
| `cuadc_fc_dryrun_0909_144100/mission_states.txt` | 话题侧状态轨迹（空：echo 晚于状态跃迁订阅，见日志 §八新坑①） |
| `cuadc_fc_dryrun_0909_144100/diagnostics.txt`、`pump.log` | mavros 诊断聚合 / GCS 心跳泵输出 |
| `cuadc_armrun_0909_152700/` | 解锁试跑·状态机路径：参数放宽 -1/0 生效 → 状态机卡 WAIT_NAV_STABLE（无 GPS 无 odom 流）→ 参数恢复 0/5 ✅ |
| `cuadc_armtest_0909_153134/` | 解锁试跑·直接路径：`arm.txt`（CommandBool → success=False）/`statustext.txt`（**Arm: Need Position Estimate** + PreArm 全列）/`state_armed.txt`（armed: false）→ 恢复 ✅ |
| `cuadc_00000012.BIN` | 飞控 SD 卡当日 dataflash 日志 134MB（电机测试 + 干跑 + 解锁试跑全程，LOG_DISARMED 连续记录），不入库仅本地 |

关键结论见 `../../09_工作日志/2026-09-09.md` §八/§九：无 GPS 时此固件无法解锁
（强制位置检查不可跳过），真解锁/全流程等 GPS（NEO-3）或 VIO。
