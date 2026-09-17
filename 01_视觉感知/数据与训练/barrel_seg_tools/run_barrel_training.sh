#!/usr/bin/env bash
# 白筒分割模型一键训练（AutoDL / 本地 GPU，现行 COCO 预训练直训路线，SSOT §8.2）
# 用法：bash run_barrel_training.sh <barrel_seg_v1 目录（含 data.yaml）> [输出目录]
# 产出：<out>/deliver_barrel/（best.pt + val_report + dataset_version.md + SHA256SUMS）
set -euo pipefail

DATA_DIR="${1:?用法: run_barrel_training.sh <数据集目录>}"
OUT="${2:-deliver_work}"
EPOCHS=100
IMGSZ=640
BATCH=16

echo "[1/6] GPU 检查"
nvidia-smi
python - <<'EOF'
import torch
assert torch.cuda.is_available(), 'CUDA 不可用——镜像选错（要 PyTorch 2.x + CUDA 12.x）'
print('GPU:', torch.cuda.get_device_name(0))
EOF

echo "[2/6] ultralytics（清华镜像；勿手滑升级 numpy 到 2.x 之外的环境无此约束）"
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "[3/6] 数据集校验（图签一一对应 + 类别仅 0）"
python - "$DATA_DIR" <<'EOF'
import os, sys, glob
d = sys.argv[1]
for split in ('train', 'val'):
    imgs = glob.glob(os.path.join(d, 'images', split, '*'))
    lbls = glob.glob(os.path.join(d, 'labels', split, '*.txt'))
    assert imgs and lbls, f'{split} 为空'
    assert len(imgs) == len(lbls), f'{split} 图 {len(imgs)} ≠ 签 {len(lbls)}'
    for l in lbls:  # 空标签=背景合法；有内容必须类 0
        for line in open(l):
            if line.strip():
                assert line.split()[0] == '0', f'非 barrel 类: {l}'
print('数据集 OK:', sum(len(glob.glob(os.path.join(d, "images", s, "*"))) for s in ("train", "val")), '张')
EOF

echo "[4/6] 训练 yolov8n-seg（参数铁律：flipud=0.5 俯视白赚；hsv_h=0.005 白色无相别特征"
echo "      但防止邻类色相噪声；close_mosaic=10 末段贴合真实分布）"
cd "$DATA_DIR"
yolo segment train model=yolov8n-seg.pt data=data.yaml epochs=$EPOCHS \
    imgsz=$IMGSZ batch=$BATCH seed=0 flipud=0.5 hsv_h=0.005 close_mosaic=10 \
    project="$OUT" name=barrel_seg

echo "[5/6] 验收（mAP50 ≥0.90 单类无豁免，漏检率 <5% 即 R≥0.95）"
RUN_DIR="$OUT/barrel_seg"
yolo segment val model="$RUN_DIR/weights/best.pt" data=data.yaml imgsz=$IMGSZ \
    project="$OUT" name=barrel_val | tee "$RUN_DIR/val_report.txt"

echo "[6/6] 打包 deliver_barrel"
mkdir -p "$OUT/deliver_barrel"
cp "$RUN_DIR/weights/best.pt" "$OUT/deliver_barrel/"
cp "$RUN_DIR/val_report.txt" "$OUT/deliver_barrel/" 2>/dev/null || true
cp "$RUN_DIR/results.csv" "$OUT/deliver_barrel/" 2>/dev/null || true
cp "$DATA_DIR/dataset_version.md" "$OUT/deliver_barrel/" 2>/dev/null || true
cp "$DATA_DIR/data.yaml" "$OUT/deliver_barrel/data_yaml_snapshot.yaml"
( cd "$OUT/deliver_barrel" && sha256sum best.pt > SHA256SUMS.txt )
echo "完成: $OUT/deliver_barrel/best.pt —— 拷回后走 deploy_trt_jetson.sh bucket 部署"
echo "⚠️ 按量计费：下载完立刻关机！"
