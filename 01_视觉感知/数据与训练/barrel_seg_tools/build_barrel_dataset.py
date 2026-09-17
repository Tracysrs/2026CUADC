#!/usr/bin/env python3
"""白筒 seg 数据集构建：合成集 + 实拍集合并、按片段防泄漏切分、自检、出交付目录。

输入约定（两路同构）：
  <src>/images/*.jpg + <src>/labels/*.txt（YOLO-seg：`0 x1 y1 x2 y2 ...` 归一化多边形）
  实拍文件名沿用 build_real_data.py 片段惯例：`<片段>_frame_*.jpg`——同片段绝不跨集。

⚠️ v1 合成集（hazard_labels_v2_11class）只有**检测框**标签，与本流水线（seg 多边形）
不兼容，勿直接混入；barrel 掩码走 render_barrel_synthetic.py 重新渲染 + 实拍标注。

用法：
  python build_barrel_dataset.py \
      --synth ../barrel_seg_tools/工作目录/synth_barrel \
      --real  ../../real_captures/barrel_v1 \
      --out   barrel_seg_v1
产出：barrel_seg_v1/{images,labels}/{train,val} + data.yaml + dataset_version.md
"""

import argparse
import os
import random
import shutil
import sys
from datetime import datetime

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def fragment_of(name):
    """片段名：NNNN_frame_xxx.jpg → NNNN；无下划线单帧 → 文件名自身。"""
    stem = os.path.splitext(name)[0]
    return stem.split('_')[0] if '_' in stem else stem


def scan(src):
    """扫描数据源：返回 [(img_path, label_path, name)]，校验图签一一对应。

    兼容两种布局：平铺 images/ + labels/，或渲染器原产的 images/{train,val} 分级
    （分级时两 split 合并后由本脚本统一按片段重切）。
    """
    img_root = os.path.join(src, 'images')
    lbl_root = os.path.join(src, 'labels')
    if not os.path.isdir(img_root):
        sys.exit(f'缺 images 目录: {src}')
    splits = [d for d in ('train', 'val') if os.path.isdir(os.path.join(img_root, d))]
    img_dirs = [os.path.join(img_root, d) for d in splits] or [img_root]
    lbl_dirs = [os.path.join(lbl_root, d) for d in splits] or [lbl_root]
    pairs, orphans = [], 0
    for img_dir, lbl_dir in zip(img_dirs, lbl_dirs):
        for name in sorted(os.listdir(img_dir)):
            if not name.lower().endswith(IMG_EXT):
                continue
            lbl = os.path.join(lbl_dir, os.path.splitext(name)[0] + '.txt')
            if os.path.isfile(lbl):
                pairs.append((os.path.join(img_dir, name), lbl, name))
            else:
                orphans += 1      # 无标签实拍帧 = 背景负样本，保留但打空标签
                pairs.append((os.path.join(img_dir, name), None, name))
    if not pairs:
        sys.exit(f'数据源为空: {src}')
    return pairs, orphans


def split_pairs(pairs, val_frac, seed):
    """按片段切分（连续帧绝不跨集——防验证泄题，SSOT §8.3）。条目 = (kind, img, lbl, name)。"""
    rng = random.Random(seed)
    frags = {}
    for item in pairs:
        name = item[3]
        frags.setdefault(fragment_of(name), []).append(item)
    keys = sorted(frags)
    rng.shuffle(keys)
    n_val = max(1, int(len(pairs) * val_frac))
    val, train, acc = [], [], 0
    for k in keys:
        if acc < n_val and len(val) + len(frags[k]) <= n_val + max(4, n_val // 10):
            val += frags[k]
            acc += len(frags[k])
        else:
            train += frags[k]
    return train, val


def validate_label(path):
    """YOLO-seg 行校验：class=0、坐标 [0,1]、点数 ≥6 且为偶数。坏行即退（防脏标）。"""
    bad = []
    with open(path, encoding='utf-8') as f:
        for ln, line in enumerate(f, 1):
            parts = line.split()
            if not parts:
                continue
            if parts[0] != '0' or len(parts) < 13 or (len(parts) - 1) % 2:
                bad.append(ln)
                continue
            vals = [float(v) for v in parts[1:]]
            if any(v < 0 or v > 1 for v in vals):
                bad.append(ln)
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--synth', default='', help='合成集目录（render_barrel_synthetic 产出）')
    ap.add_argument('--real', default='', help='实拍集目录（可多次传入前先合并）')
    ap.add_argument('--real-extra', action='append', default=[],
                    help='更多实拍集目录（可重复）')
    ap.add_argument('--out', default='barrel_seg_v1')
    ap.add_argument('--val-frac', type=float, default=0.12)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--version', default='')
    args = ap.parse_args()

    sources = []
    if args.synth:
        sources.append(('synth', args.synth))
    if args.real:
        sources.append(('real', args.real))
    for extra in args.real_extra:
        sources.append(('real', extra))
    if not sources:
        sys.exit('至少给 --synth 或 --real 之一')

    all_pairs, n_synth, n_real, n_orphan = [], 0, 0, 0
    for kind, src in sources:
        pairs, orphans = scan(src)
        all_pairs += [(kind,) + p for p in pairs]
        n_orphan += orphans
        if kind == 'synth':
            n_synth += len(pairs)
        else:
            n_real += len(pairs)
    print(f'来源：合成 {n_synth} + 实拍 {n_real}（无标签背景帧 {n_orphan}）')

    # 合成/实拍分池切分，再合并（合成独立随机 5%，实拍按片段 12%）
    synth_pairs = [p for p in all_pairs if p[0] == 'synth']
    real_pairs = [p for p in all_pairs if p[0] == 'real']
    s_train, s_val = split_pairs(synth_pairs, 0.05, args.seed)
    r_train, r_val = split_pairs(real_pairs, args.val_frac, args.seed)
    train, val = s_train + r_train, s_val + r_val

    for split in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)

    copied, empty_lbl = 0, 0
    for split, bucket in (('train', train), ('val', val)):
        for kind, img, lbl, name in bucket:
            dst_img = os.path.join(args.out, 'images', split, name)
            dst_lbl = os.path.join(args.out, 'labels', split,
                                   os.path.splitext(name)[0] + '.txt')
            shutil.copy2(img, dst_img)
            if lbl is None:
                open(dst_lbl, 'w').close()   # 空标签 = 背景图（YOLO 训练语义）
                empty_lbl += 1
            else:
                shutil.copy2(lbl, dst_lbl)
                bad = validate_label(dst_lbl)
                if bad:
                    sys.exit(f'坏标签 {lbl} 行 {bad}——修标后再建集')
            copied += 1
    if not val:
        sys.exit('val 为空——检查切分参数')

    import yaml
    with open(os.path.join(args.out, 'data.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({'names': {0: 'barrel'}, 'train': 'images/train',
                        'val': 'images/val'}, f, allow_unicode=True)

    version = args.version or f'barrel_seg_v1_{datetime.now().strftime("%Y%m%d")}'
    with open(os.path.join(args.out, 'dataset_version.md'), 'w',
              encoding='utf-8') as f:
        f.write(f'# {version}\n\n'
                f'- 合成 {n_synth} 张（render_barrel_synthetic，掩码程序化）｜'
                f'实拍 {n_real} 张（无标签背景帧 {n_orphan} → 空标签）\n'
                f'- 切分：合成随机 5% / 实拍按**片段** {args.val_frac:.0%}'
                f'（连续帧不跨集，防验证泄题）\n'
                f'- train {len(train)} / val {len(val)}，总计 {copied}，'
                f'空标签 {empty_lbl}\n'
                f'- 类别：0 = barrel（单类；三档筒径 15/20/25cm 不分类别，'
                f'直径由感知端反算对号）\n'
                f'- 建集时间 {datetime.now().strftime("%Y-%m-%d %H:%M")}，'
                f'seed={args.seed}\n')
    print(f'完成: {args.out}（train {len(train)} / val {len(val)}）')


if __name__ == '__main__':
    main()
