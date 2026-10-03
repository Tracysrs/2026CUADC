#!/usr/bin/env bash
# CUADC 2026 白筒 seg v2 · AutoDL 云 GPU 一键脚本（仅 seg；配方=v1 铁律不变）
#
# 数据（本机 barrel_work_20261002/ 打包，配方见 barrel_seg_v2/dataset_version.md）：
#   barrel_seg_v2.zip ：合成 1500（render_barrel_synthetic，v1 同批复用）
#                       + 实拍 589 = v1 实拍 162（外场+背景负样本）+ 台架新标注 427（10-01 机架帧，13 会话）
#                       ；0031 段钉 train；val 含台架段 1837/1907（新域进验证集）；zip 内含 yolov8n-seg.pt
# 复用 barrel_seg_tools 的 run_barrel_training.sh（OUT 绝对化版本）
#
# 用法：两个文件上传到 /root/autodl-tmp/ 后执行：
#   cd /root/autodl-tmp && unzip -q barrel_v2_autodl_upload.zip && bash run_barrel_v2_autodl.sh
# 环境变量可覆盖训练参数（默认 100ep/640/16 不动即可）：
#   EPOCHS=50 bash run_barrel_v2_autodl.sh
set -e

BASE=/root/autodl-tmp
EXPECT_SEG_TRAIN=1938   # 建集审计值（build_barrel_dataset.py 输出），改配方须同步
EXPECT_SEG_VAL=151

echo "===== [1/6] GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/6] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/6] 解压并校验数据集 ====="
cd "$BASE"
[ -f barrel_seg_v2.zip ] && [ ! -d barrel_seg_v2 ] && unzip -q barrel_seg_v2.zip
[ -d barrel_seg_v2 ] || { echo '!! 缺 barrel_seg_v2（zip 没传或没解压）'; exit 1; }
N_TR=$(find barrel_seg_v2/images/train -type f | wc -l)
N_VA=$(find barrel_seg_v2/images/val   -type f | wc -l)
echo "seg: train=$N_TR (期望 $EXPECT_SEG_TRAIN) val=$N_VA (期望 $EXPECT_SEG_VAL)"
[ "$N_TR" -eq "$EXPECT_SEG_TRAIN" ] && [ "$N_VA" -eq "$EXPECT_SEG_VAL" ] \
  || { echo '!! 图片数量与建集审计值不符——检查 zip 是否上传完整/配方是否改过'; exit 1; }

echo "===== [4/6] 白筒 seg 训练（run_barrel_training.sh，100ep 默认） ====="
bash run_barrel_training.sh "$BASE/barrel_seg_v2" "$BASE/deliver_work_v2"

echo "===== [5/6] 打包交付物 ====="
cd "$BASE"
rm -f deliver_barrel_v2.zip
zip -qr deliver_barrel_v2.zip deliver_work_v2/deliver_barrel
sha256sum deliver_work_v2/deliver_barrel/best.pt

echo "===== [6/6] 完成 ====="
echo "在 JupyterLab 下载交付物（右键 Download）："
echo "  /root/autodl-tmp/deliver_barrel_v2.zip （best.pt/val_report/results.csv/版本说明）"
echo "回本机先看 val_report.txt（验收线：mAP50≥0.90、R≥0.95）；best.pt 与 SHA256SUMS.txt 核对后归档。"
echo ""
echo "⚠️ 下载完立刻关机停止计费；确认交付归档后再释放实例。忘记关机是烧钱第一名。"
