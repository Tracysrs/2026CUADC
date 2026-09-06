# 飞控调试脚本（MAVLink / pymavlink + pyserial）

连接 CUAV V6X（ArduPilot 4.7-beta）用的调试小工具。2026-09-05 飞控线调试时编写，原在工作区根目录，整理后移至此处。

| 脚本 | 用途 |
|---|---|
| `check_fc.py` | 连飞控读版本 / 模式 / 电池 / GPS / 姿态 |
| `check_gps.py` | GPS 定位 / 卫星数 / 罗盘检测（室外验收：3D Fix + ≥8 星 + HDOP<1.5） |
| `verify_params.py` | `--export` 全参数备份；`--diff 参数文件` 与文件逐项比对（配置一致性检查） |
| `activate_lazy_params.py` | 激活懒加载参数组并重启（参数名探查用） |
| `read_params.py` | 读取机架 / 关键串口等指定参数（默认 COM5） |

## 使用注意

1. **先断开 Mission Planner**：它开着会占住 COM5，脚本连不上。
2. **依赖**：`pip install pymavlink pyserial`。
3. **COM 号重启后会变**：软重启可能卡 bootloader（拔插 USB 即恢复，参数不丢）；连不上先扫端口。
4. **参数导入必须两遍（中间重启）**：4.7-beta 懒加载参数组（`RNGFND1_*`、`PLND_TYPE` 等）在功能开启并重启前不存在；导入后用 `python verify_params.py --diff ../../01_设计/V6X_ardupilot_params.param` 验收。
5. 备份导出的 `param_backup_*.param` 统一放 `../../01_设计/参数备份/`。
