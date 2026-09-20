# CUAV C-RTK 3 X20P 全频 RTK 定位模块（基站+移动端一对）规格与集成定案

> **一对 = 基站（模块设为基站模式）+ 移动端（机载流动站）**。2026-09-20 采购定案立册，**承接瑞杰 RTK-01 的比赛 RTK 链路**（RTK-01-B/R 同日退役封存，rover 串口输出损坏存疑，见 [RTK-01.md](RTK-01.md)）。
> 来源：CUAV 官方文档 `doc.cuav.net/gnss/c-rtk-series/c-rtk3-x20p/` 与官网产品页（2026-09-20 查）；实物手册到货后归 `08_参考资料/手册/`，实测数据回填本册。
> **定位策略（2026-09-20 拍板）**：**验收达标后转正**——过 `06_使用说明书/04_外设链路.md` §3 的四组测试+Fix 率+带宽实测门槛前，比赛定位维持「纯 GPS+视觉」；达标后 SSOT §3.2 定案改为「RTK 主用 + 纯 GPS 自动降级兜底」。任务流程任何时候**不依赖 Fixed 硬性成立**（auto-switch 自动回 3D fix 继续飞）。

## 1. 共同规格（u-blox X20 平台）

| 参数项 | 规格 |
| --- | --- |
| GNSS 平台 | u-blox X20 全频段：GPS L1C/A、L2C、L5；GLONASS；Galileo（HAS 就绪、OSNMA 支持）；BDS；QZSS |
| 通道 / 刷新率 | 672 通道 / 25Hz |
| 处理器 | STM32H523（280MHz / 512KB Flash / 128KB RAM），模块固件 C-RTK3 M4C |
| RTK 精度 | 水平 1cm + 1ppm（流动站 + RTK 改正输入） |
| 内置罗盘 | **RM3100 工业级**（±1100μT、13nT 分辨率；数据仅经 CAN/UART 输出——本机 UART 路线**不接罗盘**） |
| 接口 | UART（JST GH 1.25 **4P**）｜CAN（JST GH 1.25 4P，1Mbit/s，FD 数据段默认 4Mbps）｜**USB Type-C**｜天线 BWMCX-KEF |
| 供电 | 4.75~5.3V @200mA（约 1W）；30g（不含天线） |
| UART 默认 | **230400、UBX 协议**（协议位掩码 bit0=UBX / bit1=NMEA / bit2=RTCM3X；USB 默认 UBX+RTCM3X） |
| 改正数据 | RTCM 3.x 输入（流动站）/ 输出（基站）；另支持 RTCM 3.4、SPARTN 2.0.2（PointPerfect PPP-RTK 需订阅，**本方案不用**） |

## 2. 基站（地面侧）

- 模块参数：`GPS_TYPE=1`（基站模式）；位置模式 `GNSS_BASE_MODE`——0=Survey-in（默认 SVIN_ACC_M=3m / SVIN_DUR_S=30s，**建议收紧并记录实际收敛值**，HIT 教训：survey-in 2.49m 勉强过线）；2=固定 LLH（`GNSS_BASE_LAT/LON`、`GNSS_BASE_H_CM` 椭球高，survey-in 收敛后**固化坐标复用**）；MSM 版本 `GNSS_RTK_MSM_VER`（**取 MSM4 + 最小消息集**，省 433 数传带宽）。
- 改正链路（本机定案，与 RTK-01 的 LoRa 自包含架构不同）：**基站 USB Type-C → 地面笔记本 Mission Planner（RTK Inject）→ 433 数传（TELEM1/SERIAL1）→ 飞控 → GPS2 移动端**。零新增无线设备、不再有 900MHz LoRa 频段；代价 = 占数传带宽，注入期间必须实测 NAV30 无 >50ms 空洞（04 册 §3）。
- 架设纪律：天线架高 1.5~2m、天顶开阔、远离反射面（多径是 Fix 收敛第一杀手）；survey-in 完成后**全程禁动**；**基站卡必建**（坐标来源 / 天线参考点 / 天线高 / 日期 / MSM 配置 / 允许架设位），换场地重新 survey-in。

## 3. 移动端（机载流动站）

- **主路线（2026-09-20 定案）：UART → V6X GPS2 口（SERIAL4）**。X20P 出厂 UBX@230400 → `GPS2_TYPE=2`(u-blox) + `SERIAL4_BAUD=230`；与 NEO-3（GPS1）组双 GPS，`GPS_AUTO_SWITCH=1`（UseBest，RTK Fixed 自动优先）。⚠️ UART 路线用不到内置 RM3100 罗盘（主罗盘仍 NEO-3 IST8310）。
- 备选路线：CAN/DroneCAN（官方推荐路径，RM3100 可用）——需与 PMU 共总线，审计 `CAN_P1_DRIVER`/节点 ID/FD 比特率对齐；**UART 实测异常（25Hz 吞吐/抗扰）时再切**，不并行折腾。
- ⚠️ **接线规则勿套用 RTK-01-R**：其「FC k 针 → rover k+1 针循环移位」只适用 RTK-01-R 的 6P 针序；X20P 是 **UART 4P**，到货按官方引脚定义页核对 + 万用表逐针（家规）。
- ⚠️ `GPS2_TYPE` **勿设 3**——本 beta 已删除 NMEA 驱动（同 04 册 §4.2 勿设 3 之坑）；X20P 默认 UBX，设 2。

## 4. 与本机集成速览

| 项 | 定案 |
| --- | --- |
| 物理接口 | 移动端 UART 4P → V6X GPS2/SERIAL4（接线卡 02 册 §9.3）；基站 USB → 地面笔记本 |
| 飞控参数 | `GPS2_TYPE=2`、`SERIAL4_BAUD=230`、`GPS_AUTO_SWITCH=1`（随到货同批两遍导入）；`GPS_BLEND` 暂关 |
| 改正链 | 基站 → MP → 433 数传 → FC → 移动端（RTCM3X，MSM4 最小消息集） |
| 罗盘 | 不用 RM3100；主罗盘仍 NEO-3 IST8310 |
| 验收 | 04 册 §3 验收分级 + 转正门槛（四组测试 / 带宽 / 静态对比） |

## 5. 到货核对清单（登记后逐项打勾，回填实测）

- [ ] 实物手册 PDF 归档 `08_参考资料/手册/`，与本文档比对（重点：UART 4P 针序、供电范围、SVIN 参数名）
- [ ] UART 4P ↔ V6X GPS2 6P 映射万用表实测（5V / TX / RX / GND 四线，TX/RX 交叉）
- [ ] 移动端出厂协议/波特率实测（应为 UBX@230400；被改过用 CUAV 工具恢复）
- [ ] 25Hz 实测吞吐（GPS2_RAW 频率）与长 burst 抗扰
- [ ] 基站模式：MP RTK Inject 全链实测 + MSM4 消息集带宽统计（NAV30 分位间隔）
- [ ] 四组验收测试（04 册 §3）记录 Fix 率 / 收敛时间 / 修正龄期 → 达标后改 SSOT §3.2 定案
