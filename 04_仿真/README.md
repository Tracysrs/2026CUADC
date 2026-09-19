# 04_仿真

仿真两条线：**SIH 台架在环仿真**（真飞控跑仿真模型）+ **Gazebo/SITL 纯仿真环境**。

| 文件/目录 | 说明 |
|---|---|
| `SIH台架/SIH台架仿真计划.md` | SIH 台架在环仿真计划：真 V6X 跑全任务 + `SIM_OH_MASK` 真舵机/电机输出，源码 pin `dbe79216` 与飞行固件同源（2026-09-15） |
| `SIH台架/SIH台架操作.md` | SIH 台架操作手册：参数备份 → 刷入 → 黑活验收 → 两档 MASK → 恢复 |
| `SIH台架/V6X_SIH_bench.param` | SIH 台架参数覆盖层 |
| `仿真环境/cuadc_rescue_sim/` | Gazebo 救援仿真环境（规则场地 + 随机场景生成；README / SETUP / CONFIGURATION / RULE_MAPPING；`models/` 第三方资产不入库） |
| `仿真环境/jetson/操作手册.md` | Jetson 上跑仿真环境的操作手册 |

SIH 固件的 Jetson 编译脚本 `sih_jetson_build.sh` 在 [`../02_飞控与硬件/调试工具/`](../02_飞控与硬件/README.md)；日常仿真四步流程与代码 SITL 联跑见 [`../06_使用说明书/07_仿真操作.md`](../06_使用说明书/07_仿真操作.md) §3 与 [`../03_机载软件/README.md`](../03_机载软件/README.md)。
