# jetson/archive · 已归档的一次性/被取代脚本

> 2026-09-25 归档（全仓审计扫尾批）。本目录脚本**不再属于现行仿真流程**（现行链路见 `../操作手册.md` 与 04_仿真/README.md），保留供排障复盘与复现历史实验。git `mv` 归档、历史可溯。

| 文件 | 原用途 | 归档原因 |
|---|---|---|
| `flight_test.py` | 早期飞行冒烟（走 sim_vehicle 14550 出口） | 与 `fly.py` 功能重叠，MAVProxy 桥接流程已被 `reset_sim.sh` 直起 SITL 取代（09-10 起） |
| `stamp_probe.py` | P0.4 时间戳对比临时探针 | 头部自述"临时探针，用完可删"；P0.4 已于 09-07 验收关闭（11 册 §13.1 留有其实证结论） |
| `fc_sitl_dryrun.sh` | 早期不解锁干跑 | 被三个全任务脚本（fc_sitl_mission/m3/m2）覆盖；07 册 §6 变体表仍留其用法（archive 路径） |
| `diag2.sh` / `diag3.sh` / `diag_takeoff.py` | 09-10 前后一次排障观测（mavros 采样/lo 抓包/daemon 预热/起飞拒止归因） | 一次性排障档案，当日结论已入工作日志与 11 册 |

不在本目录的相关件：`sr0.parm` 仍被 `start_sitl.sh`/`reset_sim.sh` defaults 链加载，**未归档**（其"SR 流率在 4.7 是否生效"存疑事项见审计记录，待定性后处置）。
