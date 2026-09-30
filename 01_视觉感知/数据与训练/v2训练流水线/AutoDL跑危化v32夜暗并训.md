# AutoDL 跑危化 v3.2 夜暗并训（hazard_v32_direct）

> 2026-10-01 备训。数据 = 哈工大重映射 + 09-29 直采 + data0930 夜暗批（516 train / 67 val），
> yolov8n COCO 直训 100ep。范式与白筒线（`../barrel_seg_tools/AutoDL跑白筒seg与H圆v1.md`）同源，
> 四条已知坑脚本均已防；本页只写本线差异。

## 1. 租实例（~10 分钟）

同白筒线 §1：按量计费 RTX 4090（24GB），镜像选 **PyTorch 2.1+ / CUDA 12.x** 官方镜像。
预估 100ep ≈ 1h 内（数据量比白筒双线大 ~2 倍，费 ~¥2~4）。

## 2. 上传（只传 1 个文件）

JupyterLab 左侧拖 `v32_autodl_upload.zip`（365.4 MB）到 `/root/autodl-tmp/`，
上传完对照 SHA256（本机出证值，**2026-10-01 凌晨重打包版**）：

```
21bbeb0272a77e09cfbf0d557865d4e434ee47c367a589f46b1feadd1630229d
```

> ⚠️ 首版总包（SHA `f73c8f8b…`）内层数据集 zip **缺顶层目录前缀**，云端解包会散落
> CWD 导致脚本 sed 报 data.yaml 不存在——已重打包修复并加结构断言。若实例上是旧包，
> 不必重传，在 `/root/autodl-tmp` 执行：
> `mkdir -p hazard_v32_direct && unzip -o -q hazard_v32_direct.zip -d hazard_v32_direct/ && rm -rf images labels data.yaml data.yaml.bak README.md && grep -q '^path:' hazard_v32_direct/data.yaml || sed -i "1i path: /root/autodl-tmp/hazard_v32_direct" hazard_v32_direct/data.yaml && bash run_v32_training.sh`

总包内含三件：`hazard_v32_direct.zip`（数据集 358.8MB）+ `yolov8n.pt`（预训练权重，
**不依赖 GitHub 下载**，被墙坑已防）+ `run_v32_training.sh`（一键脚本）。

## 3. 一键训练

```bash
cd /root/autodl-tmp && unzip -q v32_autodl_upload.zip && bash run_v32_training.sh
```

脚本自带：GPU/CUDA 检查 → 清华源装 ultralytics → 解压+**数量对账（516/67 不符拒跑）** →
直训 100ep（seed0 / imgsz640 / batch16 / patience30 / hsv_h0.005 / flipud0.5 / close_mosaic10，
**hsv_h=0.005 是铁律**：色相=类别特征，默认 0.015 会把红易燃/橙爆炸品搅混）→ 全量 val →
**夜暗 9 帧单列 val**（本批新增域，勿被整体均值稀释）→ 逐类验收判读（mAP50≥0.90 无豁免）→
打包 `deliver_v32.zip`。

**可选顺跑**：挂账的 v3.1 合训（合成+模糊）同实例可跑——上传
`hazard_v31_fusion.zip` + `run_v31_training.sh`（本目录已有），前一个跑完接着
`bash run_v31_training.sh` 即可，互不影响。

## 4. 取结果

下载 `/root/autodl-tmp/deliver_v32.zip`：best.pt / val_report.txt（全量逐类）/
val_report_night.txt（夜暗单列）/ 混淆矩阵 / 训练曲线 / data.yaml 快照 / 版本说明。
回本机后 SHA 归档 + 台账双镜像回填（`deliver_v2/dataset_version.md`）。

## 5. 停止计费（重要）

下载完**立刻关机**（不只断开 JupyterLab）；确认交付物入仓并 SHA 归档后再释放实例。
数据盘 `runs/` 中间产物可留盘备查，释放实例即清空。

## 6. 本线验收口径（训完看什么）

- 逐类 mAP50≥0.90 **全量与夜暗单列双达标**（夜暗若个别类低于线，优先看混淆矩阵
  爆炸品↔易燃行——那正是本批数据要修的边界）；
- 漏检率 R<0.95 提示行；
- 回本机后：recon_eval 不在本线范围（那是 v3.1 模糊线的裁决流程）；
  **终判 = 台架 A/B 影子复测**（1.5 牌判 0 ≥0.8×6/6 + 双牌不互串），通过前生产维持 v2。
