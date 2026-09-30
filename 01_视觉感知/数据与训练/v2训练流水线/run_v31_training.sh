#!/usr/bin/env bash
# CUADC 2026 危化标识检测 · v3.1 色移回流+运动模糊 合训（AutoDL 云 GPU 一键脚本）
#
# 路线 = yolov8n COCO 预训练直训（SSOT §8.2 口径），数据 = hazard_v31_fusion：
#   旧 11class 合成 2220（丢 barrel 线，id 原样，见 seeds/REMAP.md）
#   + 物理运动模糊 split 2000（0.5~3.5m/s 全向，2.0~3.0m 识别段高度）
#   + v3_direct 实拍 476/58（含机载直采 120，val 掺直采帧红线）
# 与 v2 的两点关键差异：直训非微调；数据绝对路径在机上是 /root/autodl-tmp 下，
# data.yaml 的 path 行由本脚本 sed 重写（v1 的坑①同款）。
#
# 用法：把 hazard_v31_fusion.zip 和本文件上传到 /root/autodl-tmp/ 后执行：
#   cd /root/autodl-tmp && bash run_v31_training.sh
set -e

BASE=/root/autodl-tmp
DATA=${BASE}/hazard_v31_fusion
EXPECT_TRAIN=4696   # 建集审计值（build_v31_dataset.py 输出），改配方须同步
EXPECT_VAL=58

echo "===== [1/6] 环境 & GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/6] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/6] 解压并校验数据集 ====="
if [ ! -d "$DATA" ]; then
  unzip -q hazard_v31_fusion.zip
fi
sed -i.bak "s|^path:.*|path: ${DATA}|" ${DATA}/data.yaml
N_TR=$(find ${DATA}/images/train -type f | wc -l)
N_VA=$(find ${DATA}/images/val -type f | wc -l)
echo "train=${N_TR} (期望 ${EXPECT_TRAIN})  val=${N_VA} (期望 ${EXPECT_VAL})"
if [ "$N_TR" -ne "$EXPECT_TRAIN" ] || [ "$N_VA" -ne "$EXPECT_VAL" ]; then
  echo "!! 图片数量与建集审计值不符——检查 zip 是否上传完整/配方是否改过"
  exit 1
fi

echo "===== [4/6] 直训（100ep，参数铁律见 SSOT §8.2 与 v2 脚本） ====="
# seed=0 复现口径；hsv_h=0.005 色相即类别特征（红=易燃 橙=爆炸品）勿用默认 0.015；
# flipud=0.5 俯视白赚增强；close_mosaic=10 尾段关 mosaic 贴合实拍分布
yolo detect train \
  model=yolov8n.pt \
  data=${DATA}/data.yaml \
  epochs=100 imgsz=640 batch=16 device=0 seed=0 patience=30 \
  hsv_h=0.005 flipud=0.5 close_mosaic=10 \
  project=${BASE}/runs name=hazard_v31_fusion

echo "===== [5/6] 验证 & 逐类验收判读 ====="
BEST=${BASE}/runs/hazard_v31_fusion/weights/best.pt
rm -rf ${BASE}/deliver && mkdir -p ${BASE}/deliver
yolo val model=${BEST} data=${DATA}/data.yaml \
  project=${BASE}/runs name=hazard_v31_fusion_val \
  > ${BASE}/deliver/val_report.txt 2>&1
tail -20 ${BASE}/deliver/val_report.txt

python - <<'PY'
# 逐类验收自动判读：mAP50 逐类 >=0.90 无豁免；召回<0.95 提示漏检超标
report = open('/root/autodl-tmp/deliver/val_report.txt', encoding='utf-8', errors='ignore').read()
rows = []
for line in report.splitlines():
    t = line.split()
    if len(t) < 7:
        continue
    try:
        vals = [float(x) for x in t[-6:]]
    except ValueError:
        continue
    rows.append((" ".join(t[:-6]), vals))
print("\n---------- 逐类验收判读 ----------")
print(f"{'类别':<12}{'P':>8}{'R':>8}{'mAP50':>8}")
bad = []
for name, (img, ins, p, r, m50, m5095) in rows:
    if name == 'all':
        continue
    flag = ''
    if m50 < 0.90:
        flag += '  <-- 低于验收线 0.90'
        bad.append(name)
    if r < 0.95:
        flag += '  <-- 漏检率超 5%'
    print(f"{name:<12}{p:>8.3f}{r:>8.3f}{m50:>8.3f}{flag}")
print("----------------------------------")
if bad:
    print(f"!! 未达标类别：{'、'.join(bad)}，先看混淆矩阵再定补数据方向")
else:
    print("全部类别达标。下一步不是直接部署——先回本机走 recon_eval replay 裁决，")
    print("再按 09-28 换模纪律上 Jetson：台架 A/B 终判（1.5 牌判 0 ≥0.8×6/6 + 双牌不互串）。")
PY

echo "===== [6/6] 打包交付物 ====="
RUN=${BASE}/runs/hazard_v31_fusion
cp ${BEST} ${BASE}/deliver/
cp ${RUN}/confusion_matrix.png ${RUN}/confusion_matrix_normalized.png ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/results.png ${RUN}/results.csv ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/val_batch0_pred.jpg ${RUN}/val_batch1_pred.jpg ${BASE}/deliver/ 2>/dev/null || true
cp ${DATA}/data.yaml ${BASE}/deliver/data_yaml_snapshot.yaml
cat > ${BASE}/deliver/dataset_version.md <<'EOF'
# hazard_v31_fusion 数据集版本说明（AutoDL 侧快照）
- 配方：旧 11class 合成 2220（丢 barrel 线，id 原样）+ 物理模糊 split 2000
  （0.5~3.5m/s 全向，2.0~3.0m 高，80% 运动/20% 清晰）+ v3_direct 实拍 476/58
  （含机载直采 120，val 掺直采帧）
- 类别：10 类新表（0爆炸品 1不燃气体 2刺激性 3放射性物品 4腐蚀品 5生物危害
  6遇湿易燃物品 7有毒品 8自燃物品 9易燃），barrel 不再作为类别
- 训练：yolov8n COCO 直训 100ep imgsz640 batch16 seed0 patience30
  hsv_h0.005 flipud0.5 close_mosaic10
- 验收线：逐类 mAP50 >= 0.90 无豁免，漏检率 < 5%；终判=台架 A/B（勿外推 val）
EOF
cd ${BASE} && zip -qr deliver_v31.zip deliver
echo "=========================================="
echo "完成！请在 JupyterLab 下载：/root/autodl-tmp/deliver_v31.zip"
echo "（best.pt / val_report.txt 逐类指标 / 混淆矩阵 / 训练曲线 / data.yaml 快照 / 版本说明）"
echo ""
echo "可选挂账：若本实例数据盘有 dataset_v4 与 best_v4.pt，可顺手补 v4 逐类 val："
echo "  yolo val model=/root/autodl-tmp/best_v4.pt data=/root/autodl-tmp/dataset_v4/data.yaml"
echo "下载完交付物后立刻关机停止计费；确认交付本仓并 SHA 归档后再释放实例。"
