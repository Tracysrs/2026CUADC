# AutoDL 跑白筒 seg + H 圆 det 第一版（barrel/hmarker v1）

> 配套：`run_barrel_v1_autodl.sh`（一键脚本，本目录）、`run_barrel_training.sh` /
> `run_hmarker_training.sh`（**必须是 2026-09-30 之后版本**，含 OUT 绝对化修复）。
> 数据配方与冒烟基线见 `barrel_work_20260930/`（本地）两份 dataset_version.md 与
> deliver_barrel/val_report.txt。路线口径 = yolov8n COCO 预训练直训（SSOT §8.2）。
> ⚠️ 本 v1 数据集（实拍仅 48/15 张）训练出的指标只证明流水线通、loss 收敛，
> **不作为验收**——SSOT M2 验收线等外场实拍 ≥600 张的正式版。

---

## 1. 租实例（~10 分钟，¥20 充值够用）

1. [autodl.com](https://www.autodl.com) → 算力市场 → **按量计费** → 显卡 **RTX 4090**（3090 也够）；
2. 镜像选框架镜像 **PyTorch 2.x + Python 3.10（CUDA 12.x）**；
3. 预计用时：seg 100ep 约 30~50 分钟 + hmarker 100ep 约 15~25 分钟 + 环境安装，
   合计 **≈1.5 小时 ≈ ¥3~5**。

## 2. 上传（JupyterLab 左侧拖到 `/root/autodl-tmp/`）

**首选：只传一个总包**（`barrel_work_20260930/barrel_v1_autodl_upload.zip`，383MB），
内含下面 5 个文件（平铺无目录层级），传完在 Terminal 解压即可：

```bash
cd /root/autodl-tmp && unzip -q barrel_v1_autodl_upload.zip
```

总包 SHA256 `f5ffeb18…40796b`（全值见打包时输出，可 `sha256sum barrel_v1_autodl_upload.zip` 核对）；
重打/改配方后重打包用 `barrel_work_20260930/_make_upload_bundle.py`（自带脚本版本校验）。

也可分传 5 文件：

| 文件 | 大小 | 说明 |
|---|---|---|
| `barrel_seg_v1.zip` | 279MB | 白筒 seg 集 1568/94（zip 内含 yolov8n-seg.pt，不用另传权重） |
| `hmarker_v1.zip` | 104MB | H 圆 det 集 110/19（zip 内含 yolov8n.pt） |
| `run_barrel_v1_autodl.sh` | 4KB | 一键脚本（本目录） |
| `run_barrel_training.sh` | 4KB | 白筒训练脚本（本目录，**新版**） |
| `run_hmarker_training.sh` | 4KB | H 圆训练脚本（本目录，**新版**） |

本机 zip 位置：`01_视觉感知/数据与训练/barrel_work_20260930/{barrel_seg_v1,hmarker_v1}.zip`
（体量原因未入库；重打单集 zip 用同目录 `_make_upload_zips.py`）。

## 3. 一键训练

JupyterLab → Terminal：

```bash
cd /root/autodl-tmp && bash run_barrel_v1_autodl.sh
```

脚本自动完成：GPU 检查 → 装 ultralytics（清华源）→ 解压 + 四项数量对账（1568/94/110/19，
与建集审计值不符即拒跑）→ 白筒 seg 训练 → H 圆 det 训练 → 两份 deliver 打包 + best.pt sha256。
训练时别关网页。要改轮数/分辨率：`EPOCHS=50 IMGSZ=640 bash run_barrel_v1_autodl.sh`。

**参数铁律**（已写死在训练脚本里，别改）：`flipud=0.5` 俯视白赚、`hsv_h=0.005`
口沿颜色不是类别特征但防邻类噪声、`close_mosaic=10` 尾段贴合实拍分布、`seed=0` 复现口径。

## 4. 取结果（两份 zip）

| 文件 | 先看什么 |
|---|---|
| `deliver_barrel_v1.zip` | `val_report.txt`（Box/Mask P/R/mAP50）、`results.png` 查过拟合、`val_batch0_pred.jpg` 目检 |
| `deliver_hmarker_v1.zip` | `val_report.txt`（19 图 7 实例的 mAP 只作方向参考，样本太少） |

本机冒烟基线（CPU 10/30ep）：Mask mAP50 0.960 / Box mAP50 0.975；H 圆 mAP50 0.898。
GPU 100ep 应当显著更高；**若 seg val 明显低于基线先查数据有没有传错（脚本数量对账已挡一层）**。

回本机：best.pt 与包内 SHA256SUMS.txt 逐字节核对（`sha256sum -c`）→ 归档 →
`dataset_version.md` 双镜像台账（deliver_v2/ 与 hazard_hgd_10cls_v2/）补条目 →
上机走 `deploy_trt_jetson.sh bucket` / `hcircle`。

## 5. 停止计费（重要）

- **下载完立刻关机**（按量计费关机即停 GPU 计费，数据保留）；
- 确认 best.pt 已归档、报告已入账，再**释放实例**（释放=数据清空）；
- 忘记关机是新手烧钱第一名，设个手机闹钟。

## 6. 已知坑（本机冒烟时踩过，AutoDL 侧脚本已防）

1. **ultralytics 8.4 相对 `project=` 路径**会挂到 `<cwd>/runs/segment/` 下 → 两个训练脚本
   2026-09-30 起把 OUT 绝对化，**别用旧版脚本**；
2. data.yaml 铁律**不写 path 键**（相对 images/train 自解析，无需 v31 的 sed）；
3. GitHub 直连不稳时权重会下载失败 → 两个预训练权重已塞进 zip，脚本不依赖外网下载模型
   （pip 走清华源不受影响）；
4. 后台训练要等完或确认孤儿进程清理——两套 run 目录并存时先对时间线再取权重
   （09-30 本机两 run 并存教训）。
