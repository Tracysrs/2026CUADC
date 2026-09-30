#!/usr/bin/env bash
# CUADC 2026 白筒 seg + H 圆 det 第一版 · AutoDL 云 GPU 一键脚本
#
# 数据（本机 barrel_work_20260930/ 打包，配方见各 dataset_version.md）：
#   barrel_seg_v1.zip ：合成 1500（render_barrel_synthetic，蓝灰各半）
#                       + 实拍 48（barrel_h_ready_20260929，两错标帧已隔离）
#                       + 背景负样本 114；0031 段钉 train；zip 内含 yolov8n-seg.pt
#   hmarker_v1.zip    ：H 圆 15 + 背景负样本 114；val=0037 整段；zip 内含 yolov8n.pt
# 复用 barrel_seg_tools 的 run_barrel_training.sh / run_hmarker_training.sh
# ⚠️ 上传的这两份必须是 2026-09-30 之后版本（含 OUT 绝对化修复），别拿旧版。
#
# 用法：五个文件上传到 /root/autodl-tmp/ 后执行：
#   cd /root/autodl-tmp && bash run_barrel_v1_autodl.sh
# 环境变量可覆盖训练参数（默认 100ep/640/16 不动即可）：
#   EPOCHS=50 bash run_barrel_v1_autodl.sh
set -e

BASE=/root/autodl-tmp
EXPECT_SEG_TRAIN=1568   # 建集审计值（build_barrel_dataset.py 输出），改配方须同步
EXPECT_SEG_VAL=94
EXPECT_HM_TRAIN=110
EXPECT_HM_VAL=19

echo "===== [1/7] GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/7] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/7] 解压并校验数据集 ====="
cd "$BASE"
[ -f barrel_seg_v1.zip ] && [ ! -d barrel_seg_v1 ] && unzip -q barrel_seg_v1.zip
[ -f hmarker_v1.zip ] && [ ! -d hmarker_v1 ] && unzip -q hmarker_v1.zip
[ -d barrel_seg_v1 ] || { echo '!! 缺 barrel_seg_v1（zip 没传或没解压）'; exit 1; }
[ -d hmarker_v1 ]    || { echo '!! 缺 hmarker_v1（zip 没传或没解压）'; exit 1; }
N_TR=$(find barrel_seg_v1/images/train -type f | wc -l)
N_VA=$(find barrel_seg_v1/images/val   -type f | wc -l)
M_TR=$(find hmarker_v1/images/train -type f | wc -l)
M_VA=$(find hmarker_v1/images/val   -type f | wc -l)
echo "seg: train=$N_TR (期望 $EXPECT_SEG_TRAIN) val=$N_VA (期望 $EXPECT_SEG_VAL)"
echo "hmarker: train=$M_TR (期望 $EXPECT_HM_TRAIN) val=$M_VA (期望 $EXPECT_HM_VAL)"
[ "$N_TR" -eq "$EXPECT_SEG_TRAIN" ] && [ "$N_VA" -eq "$EXPECT_SEG_VAL" ] \
  && [ "$M_TR" -eq "$EXPECT_HM_TRAIN" ] && [ "$M_VA" -eq "$EXPECT_HM_VAL" ] \
  || { echo '!! 图片数量与建集审计值不符——检查 zip 是否上传完整/配方是否改过'; exit 1; }

echo "===== [4/7] 白筒 seg 训练（run_barrel_training.sh，100ep 默认） ====="
bash run_barrel_training.sh "$BASE/barrel_seg_v1" "$BASE/deliver_work"

echo "===== [5/7] H 圆 det 训练（run_hmarker_training.sh，100ep 默认） ====="
bash run_hmarker_training.sh "$BASE/hmarker_v1" yolov8n.pt "$BASE/deliver_work_hmarker"

echo "===== [6/7] 打包交付物 ====="
cd "$BASE"
rm -f deliver_barrel_v1.zip deliver_hmarker_v1.zip
zip -qr deliver_barrel_v1.zip  deliver_work/deliver_barrel
zip -qr deliver_hmarker_v1.zip deliver_work_hmarker/deliver_hmarker
sha256sum deliver_work/deliver_barrel/best.pt deliver_work_hmarker/deliver_hmarker/best.pt

echo "===== [7/7] 完成 ====="
echo "在 JupyterLab 下载两份交付物（右键 Download）："
echo "  /root/autodl-tmp/deliver_barrel_v1.zip  （best.pt/val_report/results/曲线/混淆矩阵/预测样例/版本说明）"
echo "  /root/autodl-tmp/deliver_hmarker_v1.zip （best.pt/val_report/results）"
echo "回本机先看 val_report.txt；best.pt 与 SHA256SUMS.txt 核对后归档，"
echo "再走 deploy_trt_jetson.sh bucket / hcircle 上机。"
echo ""
echo "⚠️ 下载完立刻关机停止计费；确认交付归档后再释放实例。忘记关机是烧钱第一名。"
