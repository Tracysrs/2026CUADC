# SIH 台架在环仿真 · 计划与执行

> ⚠️ **状态（2026-09-17 用户拍板）：SIH 刷机取消**——真机自感知路线（飞机用自己识别到的数据控制自己），SIH 无相机图像不满足目标；飞控板保持飞行版 4.7.1-dev 未动。已编译台架固件留档 `02_飞控与硬件/firmware/`（禁外场），本计划/操作手册留作日后台架逻辑彩排的复现依据。
> **目的**：真飞控（CUAV-V6X-v2）台架上跑全任务仿真，并把仿真输出打到**真 PWM 口**——
> 真舵机按 RELEASE 时序动作、真电机跟随姿态。经典 HIL（仿真传感器注真飞控）
> ArduPilot 4.1+ 已移除，官方替代 = SITL / **SIH（Simulation on Hardware）**：
> 仿真物理模型直接跑在飞控芯片内，对外表现与 SITL 几乎一致，`SIM_OH_MASK`
> 开启后输出直驱真硬件。
> **版本锚点**：源码 pin `dbe792162d06cab66c3475fd5556bf7a120f119e`——与飞行固件
> `arducopter_beta_CUAV-V6X-v2.apj` 的 `git_identity=dbe79216`（2026-09-02 Copter
> 4.7-beta tip）同源，保证台架上验证的飞行代码与比赛日一致。
> **铁律**：① SIH 固件**只上台架，禁止外场**；② `SIM_OH_MASK` 开了电机/舵机位后
> PWM 口随时可能带输出——**上机必拆桨**；③ 参数空间与真机固件共享，切回飞行
> 固件后必须恢复参数并 diff 干净。

## 0. 与现有仿真线的关系

| 线 | 飞控代码 | 飞控硬件 | 真电机/舵机 | 用途 |
|---|---|---|---|---|
| SITL+Gazebo（现有） | ✓ | ✗ | ✗（解算值可读） | 任务算法主通道，已闭环（129s DONE / 投放 2/2） |
| **SIH（本计划）** | ✓ 同源 | ✓ V6X-v2 | ✓ `SIM_OH_MASK` 直驱 | TELEM2/MAVROS/舵机 I/O/固件行为台架彩排 |
| 台架分段（motor/servo_test） | ✓ | ✓ | ✓ | 已做完，只验单段不推进任务态 |

收益：UBEC 到货后 RELEASE 时序真舵机验证；解锁→DONE 全 18 态真飞控彩排；
外场首飞前的固件行为对照（消除"SITL 能跑真飞控没跑过"的断层）。

## 1. 阶段 A：编译 SIH 固件（无飞控参与，2026-09-15）

> **路线定案：Jetson 原生编译**。Windows 侧 Docker 路线因镜像源全部拉不动
> （ghcr denied / DaoCloud 403 / 1ms 与已配加速源 0 字节进展）废弃；WSL 发行版
> 下载同样依赖被卡的 GitHub 元数据。Jetson（JetPack6 = Ubuntu 22.04）apt 直接
> 装 `gcc-arm-none-eabi` 即可交叉编译，且其构建环境经 09-07/09-10 验证可用。
> **Windows 侧只负责：源码获取（ghfast 代理，含断点续传）→ 打包传输 → 归档。**

| # | 动作 | 产出 / 验收 |
|---|---|---|
| A1 | Windows：ghfast 代理部分克隆 pin `dbe79216`（断点续传至完整 checkout）+ 子模块 | `D:\ardupilot-sih\`，`git rev-parse HEAD` 对上锚点 |
| A2 | 本地核验：`SIM_OH_MASK` 等参数与 SIH hwdef 基底在目标 commit 存在 | grep 命中（部分已通过 raw 预检：`env SIM_ENABLED 1` / `default.param` / V6X-v2 hwdef.dat） |
| A3 | Windows 打包 `D:\ardupilot-sih-src.tar.gz`（含 .git，保版本嵌入）→ 传输 Jetson | 包体 ~200MB，Jetson 解包后 `git rev-parse` 一致 |
| A4 | Jetson 跑 `02_飞控与硬件/调试工具/sih_jetson_build.sh`（apt 依赖 → empy 3.3.4 → 官方 sitl-on-hw.py，四轴 X + Multicopter） | `~/sih_firmware_out/arducopter_SIH_CUAV-V6X-v2.apj` + SHA256 |
| A5 | 归档：产物回拷 Windows → `02_飞控与硬件/firmware/arducopter_beta_CUAV-V6X-v2_SIH_台架专用.apj` + firmware README 增行 | 文件 + README 台账同步 |

## 2. 阶段 B：台架资产

| # | 动作 | 产出 |
|---|---|---|
| B1 | SIH 参数模板 `04_仿真/SIH台架/V6X_SIH_bench.param`：`AHRS_EKF_TYPE=10`、`GPS1_TYPE=100`、`SIM_MAG1_DEVID`、`SIM_OPOS_LAT/LNG/ALT/HDG`（对齐场地）、`SIM_OH_MASK` 分级值 | 参数文件 |
| B2 | 台架操作步骤 `04_仿真/SIH台架/SIH台架操作.md`：备份→刷入→写参→MAVProxy/MAVROS 接入→跑任务→恢复 | 操作文档 |
| B3 | （可选）`ops.py` 加 `sih` 子命令：写 SIH 参数 / 恢复真机参数一键切换 | 工具集成 |

## 3. 阶段 C：上机（需人在场，**全程拆桨**）

| # | 动作 | 验收 |
|---|---|---|
| C1 | `ops.py params-export` 全量备份 → 刷 SIH 固件 → 导入 B1 模板 → 重启 | 心跳正常、模式列表正常、SIH 参数生效（回读） |
| C2 | `SIM_OH_MASK=0x300`（仅 SERVO9/10）→ 跑任务至 RELEASE | `SERVO_OUTPUT_RAW` 回读 1100/1900 逐拍跟上（对照 servo_test 基线） |
| C3 | 加电机位 → 全任务 18 态（解锁→…→DONE） | 电机跟随姿态/油门，任务态推进无卡死 |
| C4 | 重刷锁定 beta `.apj` → 恢复 C1 备份 → `verify_params --diff` | diff 干净，飞行固件回归原态 |

## 4. 风险与对策

| # | 风险 | 对策 |
|---|---|---|
| 1 | 4.7-beta tip 持续前移，SIH 编译版与飞行版行为漂移 | 永远 pin `dbe79216`；重编译换 SHA 必须同步换飞行固件并重新台架验证 |
| 2 | 参数空间共享污染真机参数 | C1 备份 + C4 恢复是硬步骤，跳过即返工 |
| 3 | `SIM_OH_MASK` 开启后电机意外转动 | 拆桨 + 安全开关在位 + 电源手动可断；mask 分级先舵机后电机 |
| 4 | V6X-v2 无 SIH 预编译固件，自编译可能有板级告警；Jetson apt 工具链（gcc-arm-none-eabi 10.3）与官方推荐版有差异 | 同源 hwdef + 同 SHA，产物先 `check_fc.py` 黑活验收再进 C2；若构建报工具链错，换 ARM 官方 10-2020-q2 tarball 重试 |
| 5 | 网络：GitHub 直连不稳（502/静默挂断） | 全部下载走 ghfast 代理；git 部分克隆可断点续传；编译在 Jetson 本地完成不依赖外网 |
