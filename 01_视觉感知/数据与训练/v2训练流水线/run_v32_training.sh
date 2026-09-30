#!/usr/bin/env bash
# CUADC 2026 危化标识检测 · v3.2 夜暗直采并训（AutoDL 云 GPU 一键脚本）
#
# 路线 = yolov8n COCO 预训练直训（SSOT §8.2 口径），数据 = hazard_v32_direct：
#   哈工大重映射 376/38（build_v3_dataset.py 产物，v2 训练域）
#   + aircraft_direct_20260929 直采 100/20（色移纠偏第一批）
#   + data0930 夜暗十类全摆场 40/9（每帧橙 1.5 与红易燃同框同光照，色移对比对）
# 与 v2 的两点关键差异：直训非微调；数据绝对路径在机上是 /root/autodl-tmp 下，
# data.yaml 的 path 行由本脚本 sed 重写（v1 的坑①同款）。
#
# 用法：把 hazard_v32_direct.zip 和本文件上传到 /root/autodl-tmp/ 后执行：
#   cd /root/autodl-tmp && bash run_v32_training.sh
set -e

BASE=/root/autodl-tmp
DATA=${BASE}/hazard_v32_direct
EXPECT_TRAIN=516   # 建集审计值（build_v32_dataset.py 输出），改配方须同步
EXPECT_VAL=67
NIGHT_STEMS="233803_000 233813_000 233824_000 233833_000 234336_000 234344_000 234359_000 234513_000 234521_000"

echo "===== [1/7] 环境 & GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/7] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/7] 解压并校验数据集 ====="
if [ ! -d "$DATA" ]; then
  unzip -q hazard_v32_direct.zip
fi
sed -i.bak "s|^path:.*|path: ${DATA}|" ${DATA}/data.yaml
N_TR=$(find ${DATA}/images/train -type f | wc -l)
N_VA=$(find ${DATA}/images/val -type f | wc -l)
echo "train=${N_TR} (期望 ${EXPECT_TRAIN})  val=${N_VA} (期望 ${EXPECT_VAL})"
if [ "$N_TR" -ne "$EXPECT_TRAIN" ] || [ "$N_VA" -ne "$EXPECT_VAL" ]; then
  echo "!! 图片数量与建集审计值不符——检查 zip 是否上传完整/配方是否改过"
  exit 1
fi

echo "===== [4/7] 直训（100ep，参数铁律见 SSOT §8.2 与 v2 脚本） ====="
# seed=0 复现口径；hsv_h=0.005 色相即类别特征（红=易燃 橙=爆炸品）勿用默认 0.015；
# flipud=0.5 俯视白赚增强；close_mosaic=10 尾段关 mosaic 贴合实拍分布
yolo detect train \
  model=yolov8n.pt \
  data=${DATA}/data.yaml \
  epochs=100 imgsz=640 batch=16 device=0 seed=0 patience=30 \
  hsv_h=0.005 flipud=0.5 close_mosaic=10 \
  project=${BASE}/runs name=hazard_v32_direct

echo "===== [5/7] 全量 val ====="
BEST=${BASE}/runs/hazard_v32_direct/weights/best.pt
rm -rf ${BASE}/deliver && mkdir -p ${BASE}/deliver
yolo val model=${BEST} data=${DATA}/data.yaml \
  project=${BASE}/runs name=hazard_v32_direct_val \
  > ${BASE}/deliver/val_report.txt 2>&1
tail -20 ${BASE}/deliver/val_report.txt

echo "===== [6/7] 夜暗 9 帧单列 val（本批新增域，逐类指标勿被整体均值稀释） ====="
NIGHT=${DATA}/night_val
rm -rf ${NIGHT} && mkdir -p ${NIGHT}/images/val ${NIGHT}/labels/val
for s in ${NIGHT_STEMS}; do
  cp ${DATA}/images/val/${s}.jpg ${NIGHT}/images/val/
  cp ${DATA}/labels/val/${s}.txt ${NIGHT}/labels/val/
done
sed "s|^path:.*|path: ${NIGHT}|; s|^train:.*|train: images/val|" ${DATA}/data.yaml > ${NIGHT}/data.yaml
yolo val model=${BEST} data=${NIGHT}/data.yaml \
  project=${BASE}/runs name=hazard_v32_night_val \
  > ${BASE}/deliver/val_report_night.txt 2>&1
tail -20 ${BASE}/deliver/val_report_night.txt

echo "===== [7/7] 逐类验收判读（全量 + 夜暗单列） ====="
python - <<'PY'
# 逐类验收自动判读：mAP50 逐类 >=0.90 无豁免；召回<0.95 提示漏检超标
def parse(path):
    report = open(path, encoding='utf-8', errors='ignore').read()
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
    return rows

def judge(rows, tag):
    print(f"\n========== 逐类验收判读（{tag}） ==========")
    print(f"{'类别':<12}{'P':>8}{'R':>8}{'mAP50':>8}")
    bad = []
    for name, (img, ins, p, r, m50, m5095) in rows:
        if name == 'all':
            print(f"{'[all]':<12}{p:>8.3f}{r:>8.3f}{m50:>8.3f}")
            continue
        flag = ''
        if m50 < 0.90:
            flag += '  <-- 低于验收线 0.90'
            bad.append(name)
        if r < 0.95:
            flag += '  <-- 漏检率超 5%'
        print(f"{name:<12}{p:>8.3f}{r:>8.3f}{m50:>8.3f}{flag}")
    print("-" * 46)
    return bad

bad_all = judge(parse('/root/autodl-tmp/deliver/val_report.txt'), '全量 67 帧')
bad_night = judge(parse('/root/autodl-tmp/deliver/val_report_night.txt'), '夜暗 9 帧单列')
if bad_all or bad_night:
    both = sorted(set(bad_all) & set(bad_night))
    print(f"!! 未达标：全量 {'、'.join(bad_all) or '无'}；夜暗 {'、'.join(bad_night) or '无'}")
    if both:
        print(f"!! 两边都未达标的 {both} = 优先补数据方向")
else:
    print("全部类别达标（含夜暗单列）。下一步不是直接部署——")
    print("按 09-28 换模纪律：台架 A/B 终判（1.5 牌判 0 ≥0.8×6/6 + 双牌不互串），通过才换产。")
PY

echo "===== 打包交付物 ====="
RUN=${BASE}/runs/hazard_v32_direct
cp ${BEST} ${BASE}/deliver/
cp ${RUN}/confusion_matrix.png ${RUN}/confusion_matrix_normalized.png ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/results.png ${RUN}/results.csv ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/val_batch0_pred.jpg ${RUN}/val_batch1_pred.jpg ${BASE}/deliver/ 2>/dev/null || true
cp ${DATA}/data.yaml ${BASE}/deliver/data_yaml_snapshot.yaml
cat > ${BASE}/deliver/dataset_version.md <<'EOF'
# hazard_v32_direct 数据集版本说明（AutoDL 侧快照）
- 配方：哈工大重映射 376/38（build_v3_dataset.py，v2 训练域，train 含 25 背景空标签负样本）
  + aircraft_direct_20260929 直采 100/20（色移纠偏第一批）
  + data0930 夜暗十类全摆场 40/9（2026-09-30 深夜，逐框目检 48 处 1.5 牌 c9→c0 修正 +
  69 杂框剔除 + 2 处贴边补框，见 data0930/README.md）
- 切分：data0930 val=233803~233833/234336~234359/234513~234521 三个互不相邻时段块（9 帧），
  块边界与相邻 train 帧 >=10s
- 类别：附件11 十类（0爆炸品 1不燃气体 2刺激性 3放射性物品 4腐蚀品 5生物危害
  6遇湿易燃物品 7有毒品 8自燃物品 9易燃）
- 训练：yolov8n COCO 直训 100ep imgsz640 batch16 seed0 patience30
  hsv_h0.005 flipud0.5 close_mosaic10
- 验收线：逐类 mAP50 >= 0.90 无豁免（夜暗 9 帧单列同线），漏检率 < 5%；
  终判=台架 A/B（勿外推 val）
EOF
cd ${BASE} && zip -qr deliver_v32.zip deliver
echo "=========================================="
echo "完成！请在 JupyterLab 下载：/root/autodl-tmp/deliver_v32.zip"
echo "（best.pt / val_report.txt 全量逐类 / val_report_night.txt 夜暗单列 / 混淆矩阵 / 训练曲线 / data.yaml 快照 / 版本说明）"
echo ""
echo "同实例可选：挂账的 v3.1 合训（合成+模糊）只需上传 hazard_v31_fusion.zip +"
echo "run_v31_training.sh 后 bash run_v31_training.sh，与本脚本互不影响。"
echo "下载完交付物后立刻关机停止计费；确认交付本仓并 SHA 归档后再释放实例。"
