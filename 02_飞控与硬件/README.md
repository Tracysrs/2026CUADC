# 02_飞控与硬件

飞控（CUAV V6X + ArduPilot 4.7-beta）参数、固件、调试工具与整机/模块设备参数。

| 文件/目录 | 说明 |
|---|---|
| `V6X_ardupilot_params.param` | V6X ArduPilot 主参数（PLND / EKF2 激光 / WPNAV 等，任务书 W3 导入，**必须导入两遍中间重启**） |
| `V6X_offboard_params.param` | offboard 相关参数快照 |
| `参数备份/` | 飞控实测参数备份：`*_20260924_231201.param`（0924，1082 项，含 EK3_RNG_USE_HGT=4 / RNGFND1_TYPE=10 / SERIAL2_BAUD=57 定案）为**当前已配置基线**（diff 对照基准），`*_195845.param`（0919，1071 项）为其前版、`*_195445.param` 为新板出厂默认；旧板备份（09-05~09-17）留档对照 |
| `firmware/` | 飞控固件存档：已刷的 4.7-beta v2 + 回退备用版 + SIH 台架专用版（附 README 与 SHA256SUMS） |
| `调试工具/README.md` | 飞控调试脚本：`ops.py` 统一入口 + check_fc / check_gps / prearm_diag / site_geo_check / rtk_setup / check_radio / verify_params / motor_test / servo_test / fc_* 全套；SIH 固件 Jetson 编译脚本 `sih_jetson_build.sh` 也在此 |
| `设备参数/C-RTK3X20P.md` | CUAV C-RTK 3 X20P 基站+移动端（09-20 起 RTK 主链，验收达标后转正） |
| `设备参数/MT-01P.md` | 微空 MT-01P 定高激光（09-20 采购定案顶替 TFmini；**09-24 实测定案 TELEM1/MAVLink/TYPE=10**） |
| `设备参数/LGC操作手册.md` | LGC 地面站操作手册（X20P 基站配置唯一工具；含"无 LGC 工作流"拍板，09-23 立册） |
| `设备参数/RTK-01.md` | （退役档案）瑞杰 RTK-01-R 规格与 6P 针序勘误（**FC k 针 → rover k+1 针循环移位，严禁 1:1 直通**；09-20 起退役封存，规则对 X20P 不适用） |
| `设备参数/ZD550.md` | 整机（ZD550 机架）设备参数调查报告 |
| `飞行日志/` | 飞控 dataflash 回拉日志存放处（`.BIN` 本地不入库）+ **架次台账**（README 内，每次回拉登记一行；命名 `YYYY-MM-DD_架次NN_目的.bin`，2026-09-19 建） |

针级接线（各座总表+逐座接线卡）见 [`../06_使用说明书/02_装机接线.md`](../06_使用说明书/02_装机接线.md)；SIH 台架仿真（计划/操作/参数）在 [`../04_仿真/SIH台架/`](../04_仿真/README.md)。
