#!/usr/bin/env bash
# CUADC 2026 危化标识检测 · v3.3 机载斜视/模糊批并训（AutoDL 云 GPU 一键脚本）
#
# 路线 = yolov8n COCO 预训练直训（SSOT §8.2 口径），配方与 v3.2 完全一致（单变量换数据），
# 数据 = hazard_v33_direct：
#   hazard_v32_direct 全量（哈工大重映射 376/38 + 09-29 直采 100/20 + data0930 夜暗 40/9）
#   + data1001 机载斜视/运动模糊/暗光批 28/10（第四域，仅 c2/c3/c4/c7/c8 五类）
# 与 v3.2 脚本差异仅两处：数据集名与数量对账值；夜暗单列改为「新域 10 帧单列」。
#
# 用法：把 v33_autodl_upload.zip 上传到 /root/autodl-tmp/ 后：
#   cd /root/autodl-tmp && unzip -q v33_autodl_upload.zip && bash run_v33_training.sh
set -e

BASE=/root/autodl-tmp
DATA=${BASE}/hazard_v33_direct
EXPECT_TRAIN=544  # 建集审计值（build_v33_dataset.py 输出），改配方须同步
EXPECT_VAL=77
NEW_STEMS="191712_015 191714_016 191716_017 191718_018 191720_019 191845_015 191847_016 191849_017 191851_018 191853_019"

echo "===== [1/7] 环境 & GPU 检查 ====="
nvidia-smi
python -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用！检查镜像是否选了 PyTorch'; print('GPU:', torch.cuda.get_device_name(0))"

echo "===== [2/7] 安装 ultralytics ====="
pip install -q ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
yolo version

echo "===== [3/7] 解压并校验数据集 ====="
if [ ! -d "$DATA" ]; then
  unzip -q hazard_v33_direct.zip
fi
sed -i.bak "s|^path:.*|path: ${DATA}|" ${DATA}/data.yaml
N_TR=$(find ${DATA}/images/train -type f | wc -l)
N_VA=$(find ${DATA}/images/val -type f | wc -l)
echo "train=${N_TR} (期望 ${EXPECT_TRAIN})  val=${N_VA} (期望 ${EXPECT_VAL})"
if [ "$N_TR" -ne "$EXPECT_TRAIN" ] || [ "$N_VA" -ne "$EXPECT_VAL" ]; then
  echo "!! 图片数量与建集审计值不符——检查 zip 是否上传完整/配方是否改过"
  exit 1
fi

echo "===== [4/7] 直训（100ep，配方与 v3.2 一致：单变量换数据） ====="
# seed=0 复现口径；hsv_h=0.005 色相即类别特征（红=易燃 橙=爆炸品）勿用默认 0.015；
# flipud=0.5 俯视白赚增强；close_mosaic=10 尾段关 mosaic 贴合实拍分布
yolo detect train \
  model=yolov8n.pt \
  data=${DATA}/data.yaml \
  epochs=100 imgsz=640 batch=16 device=0 seed=0 patience=30 \
  hsv_h=0.005 flipud=0.5 close_mosaic=10 \
  project=${BASE}/runs name=hazard_v33_direct

echo "===== [5/7] 全量 val ====="
BEST=${BASE}/runs/hazard_v33_direct/weights/best.pt
rm -rf ${BASE}/deliver && mkdir -p ${BASE}/deliver
yolo val model=${BEST} data=${DATA}/data.yaml \
  project=${BASE}/runs name=hazard_v33_direct_val \
  > ${BASE}/deliver/val_report.txt 2>&1
tail -20 ${BASE}/deliver/val_report.txt

echo "===== [6/7] 新域 10 帧单列 val（data1001 斜视/模糊/暗光，逐类指标勿被整体均值稀释） ====="
NEWD=${DATA}/newdomain_val
rm -rf ${NEWD} && mkdir -p ${NEWD}/images/val ${NEWD}/labels/val
for s in ${NEW_STEMS}; do
  cp ${DATA}/images/val/${s}.jpg ${NEWD}/images/val/
  cp ${DATA}/labels/val/${s}.txt ${NEWD}/labels/val/
done
sed "s|^path:.*|path: ${NEWD}|; s|^train:.*|train: images/val|" ${DATA}/data.yaml > ${NEWD}/data.yaml
yolo val model=${BEST} data=${NEWD}/data.yaml \
  project=${BASE}/runs name=hazard_v33_newdomain_val \
  > ${BASE}/deliver/val_report_newdomain.txt 2>&1
tail -20 ${BASE}/deliver/val_report_newdomain.txt

echo "===== [7/7] 逐类验收判读（全量 + 新域单列） ====="
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

bad_all = judge(parse('/root/autodl-tmp/deliver/val_report.txt'), '全量 77 帧')
bad_new = judge(parse('/root/autodl-tmp/deliver/val_report_newdomain.txt'), '新域 10 帧单列')
if bad_all or bad_new:
    both = sorted(set(bad_all) & set(bad_new))
    print(f"!! 未达标：全量 {'、'.join(bad_all) or '无'}；新域 {'、'.join(bad_new) or '无'}")
    if both:
        print(f"!! 两边都未达标的 {both} = 优先补数据方向（新域帧仅覆盖 c2/c3/c4/c7/c8）")
else:
    print("全部类别达标（含新域单列）。下一步不是直接部署——")
    print("按 09-28 换模纪律：台架 A/B 终判（1.5 牌判 0 ≥0.8×6/6 + 双牌不互串），通过才换产 v3.3。")
PY

echo "===== 打包交付物 ====="
RUN=${BASE}/runs/hazard_v33_direct
cp ${BEST} ${BASE}/deliver/
cp ${RUN}/confusion_matrix.png ${RUN}/confusion_matrix_normalized.png ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/results.png ${RUN}/results.csv ${BASE}/deliver/ 2>/dev/null || true
cp ${RUN}/val_batch0_pred.jpg ${RUN}/val_batch1_pred.jpg ${BASE}/deliver/ 2>/dev/null || true
cp ${DATA}/data.yaml ${BASE}/deliver/data_yaml_snapshot.yaml
cat > ${BASE}/deliver/dataset_version.md <<'EOF'
# hazard_v33_direct 数据集版本说明（AutoDL 侧快照）
- 配方：hazard_v32_direct 全量 516/67（哈工大重映射 + 09-29 直采 + data0930 夜暗，v3.2 未动）
  + data1001 机载斜视/运动模糊/暗光批 28/10（2026-09-30 傍晚无人机实拍，第四域，
  仅 c2刺激性/c3放射性/c4腐蚀品/c7有毒品/c8自燃物品 五类，逐框目检记录见 data1001/README.md）
- 切分：data1001 两连拍块（块内 2s、块间 54s）各切尾段 5 帧进 val
  （A尾 191712~191720 远距模糊 + B尾 191845~191853 近距清晰）；
  块内边界与相邻 train 帧仅隔 2s（连拍块内切割，同 v3 直采组先例）
- 类别：附件11 十类（0爆炸品 1不燃气体 2刺激性 3放射性物品 4腐蚀品 5生物危害
  6遇湿易燃物品 7有毒品 8自燃物品 9易燃）
- 训练：yolov8n COCO 直训 100ep imgsz640 batch16 seed0 patience30
  hsv_h0.005 flipud0.5 close_mosaic10（与 v3.2 配方一致，单变量换数据）
- 验收线：逐类 mAP50 >= 0.90 无豁免（新域 10 帧单列同线判读），漏检率 < 5%；
  终判=台架 A/B（勿外推 val），通过前生产维持 v3.2
EOF
cd ${BASE} && zip -qr deliver_v33.zip deliver
echo "=========================================="
echo "完成！请在 JupyterLab 下载：/root/autodl-tmp/deliver_v33.zip"
echo "（best.pt / val_report.txt 全量逐类 / val_report_newdomain.txt 新域单列 / 混淆矩阵 / 训练曲线 / data.yaml 快照 / 版本说明）"
echo ""
echo "下载完交付物后立刻关机停止计费；确认交付本仓并 SHA 归档后再释放实例。"
echo ""
echo "可选加练（同实例、互不影响）：imgsz=832 原生训练对齐 arm（台账 v3.3 选项，"
echo "推理侧 recon 配方已用 832；改两处：train 行 imgsz=832 batch=8，重跑 [4]-[7] 并"
echo "把 project/name 改成 hazard_v33_direct_832 避免覆盖）。默认不打：先保单变量。"
