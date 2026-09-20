# 微空 MicoAir MT-01P UART 激光测距模块规格与集成定案

> 2026-09-20 采购定案，**顶替 TFmini（串口版）的定高激光空位**（SSOT §3.2 定高行；TFmini 一直未到货）。占用早已预留的 TELEM3/SERIAL5 串口与参数骨架，接入改动最小。
> 来源：微空科技官网 micoair.cn 手册 / micoair.com 官方 ArduPilot 指南（2026-09-20 查）；实物手册到货后归 `08_参考资料/手册/`，实测回填本册。

## 1. 规格

| 参数项 | 规格 |
| --- | --- |
| 测距范围 | ~10m（同系 MTF-01P 标 0.02~12m@90% 反射率；MT-01P 标 ~10m，以实物手册为准） |
| 精度 | 参考 MTF-01P：4cm（0.02~2m）/ 2%（>2m） |
| 光源 | 激光测距，抗强光，室内外可用 |
| 接口 | UART（VCC / TX / RX / GND），供电 5V |
| 重量 | ~15g |
| 协议 | **多协议可配**：MicoAssistant 软件 + USB-TTL 预配置（含 **TFmini 兼容**、MAVLink 等） |

## 2. ArduPilot 集成（两条分支）

- **主路线：MicoAssistant 把 MT-01P 切 TFmini 兼容协议** → 现有参数骨架零改动沿用：`SERIAL5_PROTOCOL=11` + `SERIAL5_BAUD=115` + `RNGFND1_TYPE=20`（20=串口 Benewake 系；25 是 I2C 版勿混）。推荐理由：参数骨架 / 懒加载两遍导入流程 / 临时台账全部现成。
- 备选：MAVLink 输出 → `RNGFND1_TYPE=10`（微空官方 ArduPilot 指南口径），需改 `SERIAL5_PROTOCOL`——**仅当 TFmini 兼容模式实测有问题时切换**。
- 高度融合：`EK3_RNG_USE_HGT=4`（4m 以下激光辅助；主高度源仍 `EK3_SRC1_POSZ=1` 气压计）。

## 3. 与本机集成

| 项 | 定案 |
| --- | --- |
| 物理接口 | V6X TELEM3/SERIAL5（5V + TX/RX 交叉 + GND，接线卡 02 册 §9.6） |
| 参数 | `RNGFND1_TYPE=20`、`ORIENT=25`（下视）、`MIN=0.3`、`MAX=10`（按手册核定）、`SCALING=1`、`GNDCLR` 按实装 |
| 上游参数 | `EK3_RNG_USE_HGT` 0→4（装机验收后恢复，现临时=0） |
| 收益 | 搜索 2.0m / 投放 1.3m 真实离地高；视觉 `h=odom_z−plane_z` 解算质量（SSOT §4.2）；PLND 末段距离源；着陆确认 ≤0.30m 判据 |
| 验收 | 台架卷尺四点比对（0.3/0.6/1.3/2.0m，<2m 段 ≤4cm）；强光 / 草地 / 俯仰偏置记录；系留对比纯气压定高抖动 |

## 4. 到货核对清单

- [ ] 实物手册归档 + 与本文档比对（范围 / 精度 / 供电 / 线序）
- [ ] MicoAssistant（官网下载 + USB-TTL）确认出厂协议；切 TFmini 兼容协议并记录
- [ ] 线色实测（家规：勿凭记忆），TX/RX 交叉
- [ ] 台架四点比对 + 强光 / 深色吸收面（草地、水面）表现记录
- [ ] `MAX`/`GNDCLR` 按实装定稿；`EK3_RNG_USE_HGT=4` 两遍导入并回读
