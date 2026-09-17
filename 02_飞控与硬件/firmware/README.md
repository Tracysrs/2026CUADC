# Pixhawk V6X v2 · Copter 固件存档

> 板型已确认：**CUAV-V6X-v2**（2026-09-05 实测飞控 `board_version=0x1B59`，即 board_id 7001，
> 与本目录两版固件内的 `board_id: 7001` 一致；v1/v2 固件互不兼容，**本目录固件只用于 v2**）。

## 文件清单（2026-09-05 下载）

| 文件 | 来源 | git_identity | 用途 |
|---|---|---|---|
| `arducopter_beta_CUAV-V6X-v2.apj` | [ArduPilot 官方 beta](https://firmware.ardupilot.org/Copter/beta/CUAV-V6X-v2/arducopter.apj) | `dbe79216` | 方案 §3.2 定案：锁定验证过的 beta（先台架验证再定版）——**比赛日飞行固件** |
| `arducopter_CUAV_AP4.6.3.apj` | [CUAV 手册编译版](https://manual.cuav.net/controller/firmware/arducopter.apj) | `92b0cd78` | 备选：AP 4.6.3 稳定版，beta 出问题时回退 |
| `arducopter_beta_CUAV-V6X-v2_SIH_台架专用.apj` | 自编译（Jetson Orin Nano，xpack gcc-arm-none-eabi 10.3.1，官方 sitl-on-hw 流程，四轴 X + MultiCopter 仿真类） | `dbe79216`（同上，+SIM_ENABLED） | **SIH 台架仿真专用，禁止外场**——真飞控跑全任务+SIM_OH_MASK 真舵机/电机输出；计划见 `04_仿真/SIH台架/SIH台架仿真计划.md`，操作见 `04_仿真/SIH台架/SIH台架操作.md`；SHA256 见 `SIH固件_SHA256SUMS.txt`（2026-09-16 编译） |

> v2 板 ArduPilot 无正式版固件，官方渠道只有 beta；CUAV 手册提供 4.6.3 编译版作为稳定替代。
> 刷错板型固件 = 变砖风险：v1（Pixhawk6X）板**不要**刷本目录文件。

## 刷写步骤（Mission Planner）

1. 飞控接 USB，MP → 设置 → 安装固件 → **加载自定义固件** → 选本目录 `.apj` 文件
2. 等待烧写完成，重新插拔 USB
3. 重连后验收：
   - 心跳 `MAV_TYPE_QUADROTOR`（可用 `python check_fc.py` 验证）
   - 飞行模式下拉出现 Copter 模式（Stabilize/AltHold/PosHold/GUIDED/RTL/LAND）

## 刷机后必做（顺序）

1. `python verify_params.py --export` —— 备份默认参数
2. MP 导入 `02_飞控与硬件/V6X_ardupilot_params.param` → 写参数 → 重启
3. `python verify_params.py --diff 02_飞控与硬件/V6X_ardupilot_params.param` —— 全项一致才算过
4. 加速度计/罗盘校准（见总体方案 P1 台架步骤）
