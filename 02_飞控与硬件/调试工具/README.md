# 调试工具（按安装部件分类）

CUAV V6X（ArduPilot 4.7-beta）+ ZD550 四轴的调试脚本集，2026-09-05 飞控线调试起逐步积累。本 README 按**被调试的安装部件**分类，每个脚本标明「调试什么」。

- **统一入口**：`python ops.py` 打印分阶段命令菜单；`python ops.py <命令> [参数]` 透传调用对应脚本（.sh 会提示去 Jetson 跑）。下表「命令」列即 ops.py 命令名。
- **运行端**：`本机` = Windows COM 口直连；`Jetson` = 机载 Jetson（`/dev/cuadc-fc` + mavros 环境）；`双端` = 哪边都能跑。
- 部件规格与集成定案见 [../设备参数/](../设备参数/)（C-RTK3X20P / MT-01P / ZD550 等）。

## 使用必读（红线）

1. **先断开 Mission Planner / LGC**：它们开着会占住 COM 口，脚本连不上。
2. **依赖**：`pip install pymavlink pyserial`。
3. **COM 号重启后会变**：认端口描述不认号（飞控 = "ArduPilot MAVLink"）；连不上先扫端口。软重启可能卡 bootloader，拔插 USB 即恢复，参数不丢。
4. **参数导入必须两遍（中间重启）**：4.7-beta 懒加载参数组（`RNGFND1_*`、`PLND_TYPE` 等）在功能开启并重启前不存在；导入后必用 `verify_params.py --diff` 验收。
5. **电机/舵机类测试必须拆桨**；备份导出的 `param_backup_*.param` 统一放 [../参数备份/](../参数备份/)。

---

## 0. 统一入口

| 脚本 | 端 | 调试什么 |
|---|---|---|
| `ops.py` | 本机 | 不直接调试，**入口**：打印按《现场操作_上电到遥控手动飞行实施步骤.md》阶段分类的命令菜单并透传调用下表全部脚本 |

## 1. 飞控本体（CUAV V6X）与解锁链路

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `check_fc.py`（`fc`） | 本机 | 飞控连接自检：心跳 / 固件版本 / 机型 / 模式 / 电池 / 姿态。调试「脚本连不上飞控」「飞控现在到底啥状态」；COM 漂移时首参指定，如 `python check_fc.py COM17` |
| `fc_link_test.py` | **Jetson**（单文件可单独拷机载） | Jetson↔飞控 MAVLink 链路自检：串口占用探测 / 心跳 / 固件版本 / PING RTT / 参数点名×4 / 消息流速率判据 / GPS·姿态快照，PASS 退 0 FAIL 退 1 可接脚本门禁。默认 `/dev/ttyTHS1:921600`（TELEM3，与 bench_m2_real.sh 同源），先停 mavros 再跑；用法 `python3 fc_link_test.py [连接串] [波特率] [观测秒]` |
| `prearm_diag.py`（`prearm`） | 本机 | **解锁前门禁诊断（只读，不发解锁指令）**：模式合法性 / 油门是否低位 / RC RSSI / 电池 vs BATT_ARM_VOLT / EKF 收敛 / 罗盘一致性 / ARMING_CHECK 等门限参数 / 抓 STATUSTEXT。调试「推杆不解锁」（实测定位过 Compasses inconsistent、油门不在低位、EKF 缺位置） |
| `force_arm_skipgps.py` | 本机 | **强制解锁前置（改动式，用完必复原）**：临时 `ARMING_SKIPCHK=1280` 只跳过 GPS/位置类解锁检查（09-24 室内首飞实证值），原值存 `_arming_skipchk_orig.txt`。跳过后仅限 STABILIZE 室内手动/无桨台架，GPS 模式（GUIDED/LOITER/RTL）不可用；户外飞前必须跑 `restore_arming.py` |
| `restore_arming.py` | 本机 | **复原解锁检查**：读记录文件恢复 `ARMING_SKIPCHK` 原值（无记录回退常设值 1280；`--zero` 清零为全检查启用）。与 `force_arm_skipgps.py` 成对使用 |
| `read_params.py` | 本机 | 读机架与关键串口参数（FRAME_CLASS / FRAME_TYPE / SERIAL1_PROTOCOL / BRD_BOARD_ID），确认固件与机架配置加载正确 |

## 2. 参数系统（配置一致性）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `verify_params.py`（`params-diff`） | 本机 | 参数审计：`--export` 全量备份（刷固件/导参数前先做）；`--diff 参数文件` 与基线逐项比对。调试「配置漂移」「参数到底写进去没有」 |
| `load_params.py` | 本机 | **参数基线导入**（逐项写入+回读确认+每遍重启+自动重连，默认两遍，脚本化替代 MP 手动导入）：`python load_params.py [COM口] [参数文件]`。导完必用 `params-diff` 验收 |
| `load_params_fast.py` | 本机 | **参数快速导入（写一验一）**：每项 写入→点名读回→不符重试≤3（读回即流控），默认两遍+每遍重启。批量导入（百项级）用本件——`load_params.py` 逐项死等回执，在 compid=0 的 4.7-beta 上回执假阴性会慢到不可用（10-03 实证 1122 项单遍 ~3h）；注意链路满负荷时无回执盲写会整批丢帧（同日实证），故读回确认不可省。终判仍以 `params-diff` 实读为准 |
| `activate_lazy_params.py` | 本机 | 写入懒加载组父开关并重启，打印 RNGFND1_*/PLND_*/GUID* 完整名单。调试「参数文件里的参数在飞控上找不到 / 名字对不上」（探查新固件真实参数名） |
| `_param_dump.txt` | — | 非脚本：2026-09-13 全参数 dump 留档 |

## 3. GPS / RTK（C-RTK 3 X20P）与场地地理

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `check_gps.py`（`gps`） | 本机 | GPS/罗盘接入：定位状态 / 卫星数 / HDOP / GPS1、GPS2 参数 / 罗盘 DEV_ID / EKF。验收口径：3D Fix + ≥8 星 + HDOP<1.5 |
| `rtk_setup.py`（`rtk`） | 本机 | RTK 流动端一键配置+验证（X20P 接 GPS2）：写 GPS2_TYPE=2 + SERIAL4_BAUD=230 + GPS_AUTO_SWITCH=1 + GPS2_AUTO_CONFIG=0 + SERIAL4_PROTOCOL=5，再盯 GPS2 60s 出 RTK浮点(5)/固定(6)。⚠️ GPS2_TYPE 勿设 3（NMEA 驱动已删）；RTK 解需基站侧经 MP Inject 注入 RTCM。基站配置用 LGC（见 [../设备参数/LGC操作手册.md](../设备参数/LGC操作手册.md)） |
| `site_geo_check.py`（`geo`） | 双端 | 场地地理与定位链路就地验证（只读不飞）：`probe` 与预设场地（自贡）比对距离/方位判定「是否已在赛场」；`here` 打印实测坐标回填；`record N` 记 GPS 评估抖动漂移。⚠️ 严禁高德 GCJ-02 坐标 |

## 4. 定高激光测距（MicoAir MT-01P）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `check_rng.py`（无 ops 命令） | 本机 | 定高模块接入：SERIAL2/SERIAL5 串口配置、RNGFND1_* 参数（TYPE=0 时子参数不存在属正常）、DISTANCE_SENSOR 10Hz 实时距离流。调试「定高无数据 / 数值不对」 |

## 5. CAN 总线 / DroneCAN（X20P CAN 罗盘挂账链路）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `check_can.py`（无 ops 命令） | 本机 | CAN/DroneCAN 接入：CAN_P1_DRIVER / CAN_D1_PROTOCOL 等参数、GPS 实例、罗盘 DEV_ID/PRIO 一页汇总 + SLCAN 总线嗅探（COM16，钥匙 SERIAL8_PROTOCOL=13）。调试「X20P 的 CAN 罗盘上没上总线」 |

## 6. 数传电台（远航 X6 433，地面端）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `check_radio.py`（`radio`） | 本机 | 地面数传诊断（默认 COM10 57600，只读不改参）：链路活性（MAVLink 帧计数 = 飞控→机载台→433→地面台→PC 下行通）+ AT 命令进入 + ATI 系列身份转储。⚠️ 只发 `+++\r`（裸 +++ 无效）；**严禁 ATI5**（挂死 X-Rock 固件，只能重插 USB）；链路活跃时 AT 进不去，读/配电台先给机载端断电；MP 电台页对此电台必崩，配置走本脚本核身 + 3DR Radio Config |

## 7. 动力电机（ZD550 Quad X）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `motor_test.py`（`motor`） | 本机 | **电机顺序/方向测试（拆桨！）**：按机架换算表严格按输出口 M1→M2→M3→M4 逐个发 1 秒信号（DO_MOTOR_TEST param1 是测试序列号不是输出口）；跑前三道核验（机架换算表 / SERVO1~4_FUNCTION 绑定 / 代解硬件安全开关）+ 3 秒倒计时，任一不过拒绝测试。调试「电机转向 / 顺序 / 电调响应」 |

## 8. 投放舵机（A1 = SERVO9，A2 = SERVO10，UBEC 5V 供电）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `servo_test.py`（`servo`） | 本机 | 投放时序测试（COM 直连版）：前提核验 → 解会话安全开关 → 1500 中位→1100 收拢→1600 释放时序；回读 SERVO_OUTPUT_RAW 判读——**PWM 走了而舵机不动 = 舵机没电**（UBEC 未注入/未共地），PWM 没走 = 飞控侧问题。⚠️ 拆桨纪律 |
| `servo_position.py`（无 ops 命令） | 本机 | 舵机定位保持：打到指定 PWM（默认 SERVO9,10 @1100 收拢位）并每 5s 重发，供装盘/装爪相位安装定位。⚠️ 打 1900 = 全开会**真投放**，先取瓶 |
| `fc_servo_test.sh`（`servo-test`） | Jetson | 投放时序测试（mavros 版，PWM 口径与上同源）；`SERVO_NUM=10` 测 A2 |
| `fc_servo_diag.sh`（`servo-diag`） | Jetson | 舵机不动时的二分诊断：解安全开关 + 抓 `/mavros/rc/out` 看 PWM 是否真在变值——变了 = 飞控输出正常、问题在舵机侧电气；没变 = 输出仍被门控 |

## 9. 飞控↔Jetson 链路与任务状态机（mavros）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `fc_dryrun.sh`（`dryrun`） | Jetson | 真飞控 USB 链路一键验证 + 任务状态机干跑（无桨不解锁，仅观察 PreArm）。调试「Jetson 连飞控通不通」「状态机卡在哪一阶段」；日志落 `/tmp/cuadc_fc_dryrun_*/` |
| `fc_armrun.sh`（`armrun`） | Jetson | 解锁试跑·状态机路径：临时放宽 ARMING_SKIPCHK / FS_THR_ENABLE（trap 兜底恢复+回读校验）自动解锁尝试。调试「状态机能不能走到解锁」；无 GPS 会卡 WAIT_NAV_STABLE |
| `fc_armtest.sh`（`armtest`） | Jetson | 解锁试跑·直接路径：绕过状态机 CommandBool 解锁 10s → 上锁。调试「飞控本身肯不肯解锁」；无 GPS 被固件拒（Need Position Estimate） |
| `fc_mission_onekey.py`（`onekey`） | Jetson | **任务全流程一键启动**（09-29 新增）：冷启动自检（systemd 感知服务/磁盘 512MB/工作空间）→ MAVROS+心跳泵 → 任务节点（投放方案注入）→ 状态轨迹看护 → 结果摘要。投放两筒组合（规则 3.1.2 筒号即直径，09-29 升级）：`23`=3号+2号 400 分保底 / `13`=1号+3号 600 分跳中筒（drop_logic 新增 pair_13 档位优先序）/ `12`=1号+2号 800 分冲奖；**没有中+中**（3.1.2 仅一筒 + 6.1.2 同区算一次 + 已投筒拉黑）。**PC 端入口：`python ops.py onekey <23\|13\|12>` 自动 ssh 到 Jetson 执行**（-tt 转发 Ctrl-C）。`--no-vision` M1 演练（投放方案不生效）/ `--auto-arm` 交解锁权（默认飞手，红线）/ `--live-drop` 实弹（默认舵机干跑）/ `--dry-run` 排练 / Ctrl-C 一次退看护不杀任务（飞机可能还在飞） |
| `gcs_pump.py` | Jetson | GCS 心跳泵（先发后收）：ArduPilot 在链路上没有 GCS 心跳时不流送数据，此泵代发心跳拉起数据流。调试「mavros 起来了但没数据」；dryrun/armrun 等脚本自动调用，一般不手动跑 |

## 10. dataflash 飞行日志（SD 卡 .BIN）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `fc_log_pull.py`（`log`） | Jetson | mavftp 拉 SD 卡日志：`list` 列目录 / `get 名字` 拉取。正式拉取用 `python3 -m pymavlink.mavftp` CLI（`--burst_read_size 239`）；Windows 侧 CLI 两坑（`--baudrate` 须独立、`MSYS_NO_PATHCONV=1`）见 06 册 11 卷 §5 |
| `log_hover_check.py`（无 ops 命令） | 双端 | dataflash 离线分析（只读不动飞控）：离地窗口的姿态 / 四电机 PWM 平衡 / 振动 / 罗盘 / GPS 一页报告。调试「飞姿异常归因」（如左倾 = 机械不平 / 推力不平衡 / 操纵补偿），`python log_hover_check.py <日志.BIN>`；加 `--fft` 附发 IMU 角速度频谱+峰频表（纯 Python FFT，低频段振荡/整定前后对比用；电机段陷波定频用 MP 日志 FFT，见 03 册 §2.10） |

## 11. 台架联测（带电投放记录）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `bench_logger.py`（无 ops 命令） | Jetson | 电压记录仪：10Hz 记电压/油门/急停/解锁态到 `/tmp/cuadc_bench*.csv`。⚠️ 与投放脚本**不可同跑**（同读串口互抢） |
| `bench_drop.py`（无 ops 命令） | Jetson | 投放执行联测（单进程独占串口）：等解锁 → 怠速稳定 2s → 自动释放/回仓，电压与事件打点进 CSV。调试「带电投放全链路」（电压跌落 / 时序） |

## 12. 固件（SIH 台架）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `sih_jetson_build.sh` | Jetson | SIH 台架固件一键原生编译（xpack 工具链，pin dbe79216），产物归 [../firmware/](../firmware/)。⚠️ 产物只上台架，**禁止外场**。SIH 操作见 [../../04_仿真/SIH台架/SIH台架操作.md](../../04_仿真/SIH台架/SIH台架操作.md) |
| `recon_bench_onekey.sh` | Jetson | **判读基准验证一键脚本**（09-25 新增，05 册 §6.1 的执行件）：环境→相机→单实例→bench→连发递增请求→**计时收结果**（窗=4s 证据窗，实测延迟 6~7s/窗）。`bash recon_bench_onekey.sh [窗数] [views]`；`stop` 收尾自动恢复 systemd；seq 状态存 `/tmp/recon_bench_onekey.seq` 跨运行递增（防同号去重忽略） |

## 13. 仓库↔Jetson 同步（代码一致性）

| 脚本（命令） | 端 | 调试什么 |
|---|---|---|
| `sync_jetson.py`（`sync`） | 本机 | **仓库→Jetson 一键同步（只推不拉）**：清单化映射 + 逐文件 md5 比对。`push [--yes]` 推送 / `diff 关键词` 看单文件差异 / `manifest` 冻结证据链。调试/治理「仓库与机侧代码静默漂移」；Jetson 无本仓克隆，此工具是唯一同步通道 |

## 附：留档目录

- `jetson_fc_dryrun_2026-09-09/` —— 首次真飞控 dryrun 的日志留档（非脚本）。
- `__pycache__/` —— Python 缓存，忽略。
