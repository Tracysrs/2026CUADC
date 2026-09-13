#!/usr/bin/env bash
# CUADC 2026 危化标识检测 · v2 实拍数据混合微调（AutoDL 云 GPU 一键脚本）
#
# ⚠️ 2026-09-12 状态：本脚本链路【停用】——上游 v1 合成集与 real_data 标签均为
# 旧 GHS 类别表（id 2~5 与附件11 实表错位，见 01_视觉模块 §2.1）。
# 现行路线 = yolov8n COCO 预训练直训（deliver_v2 已验证，见 SSOT §8.2）。
# 数据重映射后如需复活微调链，本脚本可继续使用（验收判读已改逐类无豁免）。
#
# 对应任务书：PC端训练任务书.md §6（验收线：逐类 mAP50>=0.90 无豁免，漏检率<5%）
# 用法：把 real_data.zip 和本文件上传到 /root/autodl-tmp/ 后执行：
#   cd /root/autodl-tmp && bash run_v2_training.sh
#
# 关于 v1 权重（微调起点），脚本按以下顺序自动寻找，找到即用：
#   1) /root/autodl-tmp/v1_best.pt（推荐：把上次交付的 best.pt 改名后上传）
#   2) /root/autodl-tmp/best.pt
#   3) /root/autodl-tmp/deliver/best.pt 或 deliver_v1.zip（自动解压提取）
#   4) /root/autodl-tmp/runs/hazard_synth_v1/weights/best.pt（本实例已跑过 v1）
#   5) 都没有、但上传了 hazard_labels_v2_11class.tar.gz → 先自动训 v1 再微调（全程约 1 小时）
set -e

BASE=/root/autodl-tmp
cd "$BASE"

echo "===== [1/7] 环境 & GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/7] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/7] 解压实拍数据集 ====="
if [ ! -d real_data ]; then
  unzip -q real_data.zip
fi
N=$(find real_data/images -type f | wc -l)
echo "实拍图片数：$N（应为 414：train 376 + val 38）"
echo "实拍标签数：$(find real_data/labels -name '*.txt' | wc -l)（应为 414，含 25 个背景空标签）"
if [ "$N" -ne 414 ]; then
  echo "!! 图片数量不对，检查 real_data.zip 是否上传完整"
  exit 1
fi
# data.yaml 无 path 键，ultralytics 以 yaml 所在目录为根，无需修正路径（v1 的坑①不存在）

echo "===== [4/7] 定位 v1 权重（微调起点） ====="
V1_BEST=""
for cand in v1_best.pt best.pt deliver/best.pt runs/hazard_synth_v1/weights/best.pt; do
  if [ -f "$BASE/$cand" ]; then V1_BEST="$BASE/$cand"; break; fi
done
if [ -z "$V1_BEST" ] && [ -f deliver_v1.zip ]; then
  echo "从 deliver_v1.zip 提取 v1 权重……"
  unzip -o -q deliver_v1.zip
  [ -f deliver/best.pt ] && V1_BEST="$BASE/deliver/best.pt"
fi
if [ -z "$V1_BEST" ] && [ -f hazard_labels_v2_11class.tar.gz ]; then
  echo "没找到 v1 权重，先用合成数据训练 v1（约 10~30 分钟）……"
  if [ ! -d hazard_labels ]; then
    tar -xzf hazard_labels_v2_11class.tar.gz
  fi
  sed -i.bak "s|^path:.*|path: ${BASE}/hazard_labels/synth|" hazard_labels/synth/data.yaml
  echo "合成图片数：$(find hazard_labels/synth/images -name '*.jpg' | wc -l)（应为 2220）"
  yolo detect train \
    model=yolov8n.pt \
    data=${BASE}/hazard_labels/synth/data.yaml \
    epochs=100 imgsz=640 batch=16 device=0 patience=30 \
    project=${BASE}/runs name=hazard_synth_v1
  V1_BEST=${BASE}/runs/hazard_synth_v1/weights/best.pt
fi
if [ -z "$V1_BEST" ]; then
  echo "!! 没找到 v1 权重（任务书 §6：微调必须从 v1 best.pt 起步）。三选一："
  echo "   a. 上传上次 v1 的 deliver_v1.zip（或其中 best.pt 改名为 v1_best.pt）"
  echo "   b. 上传 best.pt 到 $BASE/"
  echo "   c. 连 hazard_labels_v2_11class.tar.gz 一起上传，本脚本自动先训 v1"
  exit 1
fi
echo "微调起点：$V1_BEST"

echo "===== [5/7] 开始 v2 微调（任务书 §6 参数铁律，约 15~40 分钟） ====="
# lr0=0.001      微调在预训练权重上学习率降 10 倍
# flipud=0.5     俯视图上下翻转是白赚的增强
# hsv_h=0.005    色相就是类别特征（红=易燃 橙=爆炸品），必须低于默认 0.015
# close_mosaic=10 最后 10 轮关 mosaic 贴合真实分布
yolo detect train \
  model=${V1_BEST} \
  data=${BASE}/real_data/data.yaml \
  epochs=150 imgsz=640 batch=16 device=0 patience=50 \
  lr0=0.001 flipud=0.5 hsv_h=0.005 close_mosaic=10 \
  project=${BASE}/runs name=hazard_mix_v2

echo "===== [6/7] 验证 & 逐类验收判读 ====="
BEST=${BASE}/runs/hazard_mix_v2/weights/best.pt
rm -rf ${BASE}/deliver && mkdir -p ${BASE}/deliver
yolo val model=${BEST} data=${BASE}/real_data/data.yaml \
  project=${BASE}/runs name=hazard_mix_v2_val \
  > ${BASE}/deliver/val_report.txt 2>&1
tail -20 ${BASE}/deliver/val_report.txt

python - <<'PY'
# 逐类验收自动判读：mAP50 逐类 >=0.90 无豁免（09-12 勘正，取消旧"白底三类>=0.85"豁免）；
# 召回<0.95 提示漏检超标
white_bg = set()  # 保留变量位：旧表豁免集合已随类别表勘正取消
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
print(f"{'类别':<10}{'P':>8}{'R':>8}{'mAP50':>8}")
bad = []
for name, (img, ins, p, r, m50, m5095) in rows:
    if name == 'all':
        continue
    thr = 0.85 if name in white_bg else 0.90
    flag = ''
    if m50 < thr:
        flag += f'  <-- 低于验收线 {thr}'
        bad.append(name)
    if r < 0.95:
        flag += '  <-- 漏检率超 5%'
    print(f"{name:<10}{p:>8.3f}{r:>8.3f}{m50:>8.3f}{flag}")
print("----------------------------------")
if bad:
    print(f"!! 未达标类别：{'、'.join(bad)}，先看混淆矩阵确认和谁互混，再决定补采哪类数据")
else:
    print("全部类别达标，可交付 Jetson")
PY

echo "===== [7/7] 打包交付物 ====="
RUN=${BASE}/runs/hazard_mix_v2
cp ${BEST} ${BASE}/deliver/
cp ${RUN}/confusion_matrix.png ${RUN}/confusion_matrix_normalized.png ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/results.png ${RUN}/results.csv ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/val_batch0_pred.jpg ${RUN}/val_batch1_pred.jpg ${BASE}/deliver/ 2>/dev/null || true
cp ${BASE}/real_data/data.yaml ${BASE}/deliver/real_data_data_yaml_snapshot.yaml
cat > ${BASE}/deliver/dataset_version.md <<'EOF'
# hazard_mix_v2 数据集版本说明
- 来源：Jetson 端实拍（无人机俯拍草地/路面上贴危化标识的桶）
- 规模：414 张（train 376 含 25 张背景图 / val 38），实例约 2376
- 切分：按视频片段整段切（frame_* 取中段连续帧、1_* 取末段、101 个单帧片段随机 10%），避免连续帧泄题
- 类别：0~9 十个标识类 + 10 barrel（本批无 barrel 实例，保留类目以对齐 v1 检测头）
- 验收线：逐类 mAP50 >= 0.90（逐类无豁免），漏检率 < 5%
EOF
cd ${BASE} && zip -qr deliver_v2.zip deliver
echo "=========================================="
echo "完成！请在 JupyterLab 下载：/root/autodl-tmp/deliver_v2.zip"
echo "（内含 best.pt / val_report.txt 逐类指标 / 混淆矩阵 / 训练曲线 / 预测样例 / 数据集版本说明）"
echo "下载完后请关机停止计费；确认交付 Jetson 后再释放实例。"
