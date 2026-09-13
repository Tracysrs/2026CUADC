# 飞控调试脚本（MAVLink / pymavlink + pyserial）

连接 CUAV V6X（ArduPilot 4.7-beta）用的调试小工具。2026-09-05 飞控线调试时编写，原在工作区根目录，整理后移至此处。

| 脚本 | 用途 |
|---|---|
| `ops.py` | **统一入口**：`python ops.py` 打印按《现场操作清单》阶段分类的命令菜单；`python ops.py <命令> [参数]` 透传调用下表脚本（.sh 自动提示去 Jetson 跑） |
| `check_fc.py` | 连飞控读版本 / 模式 / 电池 / GPS / 姿态 |
| `check_gps.py` | GPS 定位 / 卫星数 / 罗盘检测（室外验收：3D Fix + ≥8 星 + HDOP<1.5）。2026-09-13 修 pymavlink 新版 VDOP 字段兼容 |
| `prearm_diag.py` | 解锁前诊断（阶段0）：模式/RC/电池/EKF/罗盘一致性/参数门限一次汇总 + 抓 STATUSTEXT，只读不发解锁指令。2026-09-13 实战定位 "PreArm: Compasses inconsistent" |
| `site_geo_check.py` | 场地地理数据（CONFIG 区预填自贡航空产业园：29.3765N/104.6258E/标高345m 占位）：probe 就地测试 / here 实测回填 / record N 记 GPS。严禁用高德 GCJ-02 坐标 |
| `rtk_setup.py` | RTK rover（RTK-01-R 接 GPS2）一键配置+验证：GPS2_TYPE=3(NMEA) + **SERIAL4_BAUD=921**（本版 4.7 无 GPS2_BAUD 参数，见 11_设备参数/RTK-01.md §6） |
| `check_radio.py` | 地面数传诊断（默认 COM10）：链路活性（MAVLink 帧计数）+ AT 命令模式进入 + ATI 系列身份转储，只读不改参。**只发 `+++\r`**（远航 X6 = X-Rock 4.0 固件，裸 `+++` 无效）；**严禁 ATI5**（挂死 X-Rock 固件，只能重插 USB 恢复）。链路活跃时 AT 进不去（心跳流打断静默窗口），读/配电台先给机载端断电。MP 1.3.83 电台页对此电台必崩"端口被关闭"，配置走脚本核身 + 3DR Radio Config（见 2026-09-12 日志） |
| `verify_params.py` | `--export` 全参数备份；`--diff 参数文件` 与文件逐项比对（配置一致性检查） |
| `activate_lazy_params.py` | 激活懒加载参数组并重启（参数名探查用） |
| `read_params.py` | 读取机架 / 关键串口等指定参数（默认 COM5） |
| `motor_test.py` | 电机顺序/方向测试（拆桨用）。**严格按输出口 M1→M2→M3→M4** 逐个发 1 秒信号：`python motor_test.py [COM口] [输出列表]`，如 `python motor_test.py COM5 3` 只转 M3。跑前三道核验（机架→测试序换算表 / SERVO1~4_FUNCTION=Motor1~4 绑定 / 代解硬件安全开关）+ 3 秒倒计时，任一不过拒绝测试（见 2026-09-09 日志 §三/§五、2026-09-11 日志） |
| `fc_dryrun.sh` | **在 Jetson 上跑**：真飞控 USB 链路一键验证 + 任务状态机干跑（无桨不解锁）。`bash fc_dryrun.sh [秒数]`，日志落 `/tmp/cuadc_fc_dryrun_*/`。首次实跑见 2026-09-09 日志 §八 |
| `fc_log_pull.py` | **在 Jetson 上跑**：mavftp 拉飞控 SD 卡 dataflash 日志，`list` 列目录 / `get 名字` 拉取。注意 `cmd_get` 异步，正式拉取用 `python3 -m pymavlink.mavftp --device /dev/cuadc-fc --burst_read_size 239 get /APM/Logs/x.BIN 本地路径`（见 2026-09-09 日志 §八） |
| `fc_armrun.sh` | **在 Jetson 上跑**：解锁试跑·状态机路径。临时放宽 ARMING_SKIPCHK/FS_THR_ENABLE（trap 兜底恢复+回读校验），任务状态机自动解锁尝试。无 GPS 时会卡 WAIT_NAV_STABLE（见 2026-09-09 日志 §九） |
| `fc_armtest.sh` | **在 Jetson 上跑**：解锁试跑·直接路径。绕过状态机直接 CommandBool 解锁 10s → 上锁。无 GPS 时被固件强制位置检查拒绝（`Arm: Need Position Estimate`），参数同样临时改+自动恢复 |
| `fc_servo_test.sh` | **在 Jetson 上跑**：舵机投放时序（DO_SET_SERVO，收拢 1100/释放 1900 与任务逻辑同源），自动解会话安全锁 + SERVOx_FUNCTION 检查。`SERVO_NUM=10` 测 A2。舵机不动的根因排查见 2026-09-09 日志 §舵机排障（系统无 5V 源） |
| `fc_servo_diag.sh` | **在 Jetson 上跑**：舵机不动时的诊断——解会话安全锁 + 抓 `/mavros/rc/out` 看飞控输出通道是否真在变值（区分"指令没到"与"舵机没电"） |

## 使用注意

1. **先断开 Mission Planner**：它开着会占住 COM5，脚本连不上。
2. **依赖**：`pip install pymavlink pyserial`。
3. **COM 号重启后会变**：软重启可能卡 bootloader（拔插 USB 即恢复，参数不丢）；连不上先扫端口。
4. **参数导入必须两遍（中间重启）**：4.7-beta 懒加载参数组（`RNGFND1_*`、`PLND_TYPE` 等）在功能开启并重启前不存在；导入后用 `python verify_params.py --diff ../../01_设计/V6X_ardupilot_params.param` 验收。
5. 备份导出的 `param_backup_*.param` 统一放 `../../01_设计/参数备份/`。
