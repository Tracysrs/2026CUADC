# 飞控调试脚本（MAVLink / pymavlink + pyserial）

连接 CUAV V6X（ArduPilot 4.7-beta）用的调试小工具。2026-09-05 飞控线调试时编写，原在工作区根目录，整理后移至此处。

| 脚本 | 用途 |
|---|---|
| `check_fc.py` | 连飞控读版本 / 模式 / 电池 / GPS / 姿态 |
| `check_gps.py` | GPS 定位 / 卫星数 / 罗盘检测（室外验收：3D Fix + ≥8 星 + HDOP<1.5） |
| `verify_params.py` | `--export` 全参数备份；`--diff 参数文件` 与文件逐项比对（配置一致性检查） |
| `activate_lazy_params.py` | 激活懒加载参数组并重启（参数名探查用） |
| `read_params.py` | 读取机架 / 关键串口等指定参数（默认 COM5） |
| `motor_test.py` | 电机顺序/方向测试（拆桨用）。按输出口 M1~M4 逐个转 1 秒：`python motor_test.py [COM口] [输出列表]`，如 `python motor_test.py COM5 3` 只转 M3。内部已做输出口→测试序列号换算（见 2026-09-09 日志的坑） |
| `fc_dryrun.sh` | **在 Jetson 上跑**：真飞控 USB 链路一键验证 + 任务状态机干跑（无桨不解锁）。`bash fc_dryrun.sh [秒数]`，日志落 `/tmp/cuadc_fc_dryrun_*/`。首次实跑见 2026-09-09 日志 §八 |
| `fc_log_pull.py` | **在 Jetson 上跑**：mavftp 拉飞控 SD 卡 dataflash 日志，`list` 列目录 / `get 名字` 拉取。注意 `cmd_get` 异步，正式拉取用 `python3 -m pymavlink.mavftp --device /dev/cuadc-fc --burst_read_size 239 get /APM/Logs/x.BIN 本地路径`（见 2026-09-09 日志 §八） |
| `fc_armrun.sh` | **在 Jetson 上跑**：解锁试跑·状态机路径。临时放宽 ARMING_SKIPCHK/FS_THR_ENABLE（trap 兜底恢复+回读校验），任务状态机自动解锁尝试。无 GPS 时会卡 WAIT_NAV_STABLE（见 2026-09-09 日志 §九） |
| `fc_armtest.sh` | **在 Jetson 上跑**：解锁试跑·直接路径。绕过状态机直接 CommandBool 解锁 10s → 上锁。无 GPS 时被固件强制位置检查拒绝（`Arm: Need Position Estimate`），参数同样临时改+自动恢复 |

## 使用注意

1. **先断开 Mission Planner**：它开着会占住 COM5，脚本连不上。
2. **依赖**：`pip install pymavlink pyserial`。
3. **COM 号重启后会变**：软重启可能卡 bootloader（拔插 USB 即恢复，参数不丢）；连不上先扫端口。
4. **参数导入必须两遍（中间重启）**：4.7-beta 懒加载参数组（`RNGFND1_*`、`PLND_TYPE` 等）在功能开启并重启前不存在；导入后用 `python verify_params.py --diff ../../01_设计/V6X_ardupilot_params.param` 验收。
5. 备份导出的 `param_backup_*.param` 统一放 `../../01_设计/参数备份/`。
