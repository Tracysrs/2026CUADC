#!/usr/bin/env bash
# H 圆（h_marker）单类检测模型一键训练（AutoDL / 本地 GPU）
# 数据：自制 ø80cm H 标 ~200 张（采集卡见 01_视觉感知/数据与训练/barrel_seg_tools/采集操作卡.md）
# 初值路线：有 helipad 公开集预训权重则传入作为起点，没有直接 COCO yolov8n 起（SSOT §8.1）
# 用法：bash run_hmarker_training.sh <数据集目录（含 data.yaml，单类 0=h_marker）> [helipad.pt] 
set -euo pipefail

DATA_DIR="${1:?用法: run_hmarker_training.sh <数据集目录> [helipad预训.pt]}"
PRETRAINED="${2:-yolov8n.pt}"
OUT="${3:-deliver_work_hmarker}"
EPOCHS=100
IMGSZ=640
BATCH=16

echo "[1/6] GPU 检查"
nvidia-smi
python - <<'EOF'
import torch
assert torch.cuda.is_available(), 'CUDA 不可用'
print('GPU:', torch.cuda.get_device_name(0))
EOF

echo "[2/6] ultralytics"
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version
if [ "$PRETRAINED" != "yolov8n.pt" ] && [ ! -f "$PRETRAINED" ]; then
  echo "预训权重不存在: $PRETRAINED"; exit 1
fi

echo "[3/6] 数据集校验（单类 0=h_marker）"
python - "$DATA_DIR" <<'EOF'
import os, sys, glob
d = sys.argv[1]
for split in ('train', 'val'):
    imgs = glob.glob(os.path.join(d, 'images', split, '*'))
    lbls = glob.glob(os.path.join(d, 'labels', split, '*.txt'))
    assert imgs and lbls and len(imgs) == len(lbls), f'{split} 图签不齐'
    for l in lbls:
        for line in open(l):
            if line.strip():
                assert line.split()[0] == '0', f'非 h_marker 类: {l}'
print('数据集 OK')
EOF

echo "[4/6] 训练 yolov8n 检测（H 圆是大目标，640 足够；俯视无上下语义 flipud=0.5）"
cd "$DATA_DIR"
yolo detect train model="$PRETRAINED" data=data.yaml epochs=$EPOCHS \
    imgsz=$IMGSZ batch=$BATCH seed=0 flipud=0.5 close_mosaic=10 \
    project="$OUT" name=hmarker_det

echo "[5/6] 验收（mAP50 ≥0.90，R ≥0.95；降落是 50 分项，漏检即丢目标）"
RUN_DIR="$OUT/hmarker_det"
yolo detect val model="$RUN_DIR/weights/best.pt" data=data.yaml imgsz=$IMGSZ \
    project="$OUT" name=hmarker_val | tee "$RUN_DIR/val_report.txt"

echo "[6/6] 打包 deliver_hmarker"
mkdir -p "$OUT/deliver_hmarker"
cp "$RUN_DIR/weights/best.pt" "$OUT/deliver_hmarker/"
cp "$RUN_DIR/val_report.txt" "$OUT/deliver_hmarker/" 2>/dev/null || true
cp "$RUN_DIR/results.csv" "$OUT/deliver_hmarker/" 2>/dev/null || true
( cd "$OUT/deliver_hmarker" && sha256sum best.pt > SHA256SUMS.txt )
echo "完成: $OUT/deliver_hmarker/best.pt —— 拷回后走 deploy_trt_jetson.sh hcircle 部署"
echo "⚠️ 按量计费：下载完立刻关机！"
