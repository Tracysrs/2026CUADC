#!/usr/bin/env python3
"""H 圆（h_marker）单类检测数据集构建：实拍正样本 + 背景负样本、按片段防泄漏切分。

与 build_barrel_dataset.py 同一套片段纪律（同片段绝不跨 train/val），差异：
  * 标签是 YOLO **det**（`0 cx cy w h`，5 字段）——build_barrel_dataset 的 seg 校验
    （≥6 点）不兼容，本脚本按 det 口径校验（含框边缘回读 [0,1] 检查）；
  * 支持 --val-fragments 把指定片段整段钉进 val（正样本少的集若让贪心随机决定，
    val 可能一张正样本都没有，val_report 失去意义）。

用法：
  python build_hmarker_dataset.py \
      --real <H 圆标注集>（images/ + labels/） \
      --real-extra <背景负样本集>（可重复） \
      --val-fragments 0037 --pin-train 0031 --out hmarker_v1
产出：hmarker_v1/{images,labels}/{train,val} + data.yaml（单类 0=h_marker）+ dataset_version.md
"""

import argparse
import os
import random
import shutil
import sys
from datetime import datetime

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def fragment_of(name):
    stem = os.path.splitext(name)[0]
    return stem.split('_')[0] if '_' in stem else stem


def scan(src):
    img_root = os.path.join(src, 'images')
    lbl_root = os.path.join(src, 'labels')
    if not os.path.isdir(img_root):
        sys.exit(f'缺 images 目录: {src}')
    pairs, orphans = [], 0
    for name in sorted(os.listdir(img_root)):
        if not name.lower().endswith(IMG_EXT):
            continue
        lbl = os.path.join(lbl_root, os.path.splitext(name)[0] + '.txt')
        if os.path.isfile(lbl):
            pairs.append((os.path.join(img_root, name), lbl, name))
        else:
            orphans += 1          # 无标签帧 = 背景负样本，保留但打空标签
            pairs.append((os.path.join(img_root, name), None, name))
    if not pairs:
        sys.exit(f'数据源为空: {src}')
    return pairs, orphans


def validate_label(path):
    """YOLO det 行校验：class=0、5 字段、值 [0,1]、框边缘回读 [0,1]（1e-6 容差）。"""
    bad = []
    with open(path, encoding='utf-8') as f:
        for ln, line in enumerate(f, 1):
            parts = line.split()
            if not parts:
                continue
            if parts[0] != '0' or len(parts) != 5:
                bad.append(ln)
                continue
            cx, cy, bw, bh = (float(v) for v in parts[1:])
            if any(v < 0 or v > 1 for v in (cx, cy, bw, bh)):
                bad.append(ln)
                continue
            if (cx - bw / 2 < -1e-6 or cx + bw / 2 > 1 + 1e-6
                    or cy - bh / 2 < -1e-6 or cy + bh / 2 > 1 + 1e-6):
                bad.append(ln)
    return bad


def split_pairs(pairs, val_frac, seed, pin_train=(), pin_val=()):
    """按片段切分；钉扎片段优先（val 钉扎计入 val 配额，不足额不再补）。"""
    rng = random.Random(seed)
    frags = {}
    for item in pairs:
        frags.setdefault(fragment_of(item[2]), []).append(item)
    missing = [k for k in (*pin_train, *pin_val) if k not in frags]
    if missing:
        print(f'⚠️ 钉扎片段在本池中不存在（忽略）: {missing}')
    n_val = max(1, int(len(pairs) * val_frac))
    val, acc = [], 0
    for k in pin_val:
        if k in frags:
            val += frags[k]
            acc += len(frags[k])
    train = [it for k in pin_train if k in frags for it in frags[k]]
    keys = [k for k in sorted(frags) if k not in pin_train and k not in pin_val]
    rng.shuffle(keys)
    for k in keys:
        if acc < n_val and len(val) + len(frags[k]) <= n_val + max(4, n_val // 10):
            val += frags[k]
            acc += len(frags[k])
        else:
            train += frags[k]
    return train, val


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--real', default='', help='H 圆正样本集目录')
    ap.add_argument('--real-extra', action='append', default=[],
                    help='更多来源（背景负样本等，可重复）')
    ap.add_argument('--out', default='hmarker_v1')
    ap.add_argument('--val-frac', type=float, default=0.12)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--pin-train', action='append', default=[],
                    help='整段钉死在 train 的片段号（可重复）')
    ap.add_argument('--val-fragments', action='append', default=[],
                    help='整段钉死在 val 的片段号（可重复，建议给正样本最多的段）')
    ap.add_argument('--version', default='')
    args = ap.parse_args()

    sources = []
    if args.real:
        sources.append(args.real)
    sources += args.real_extra
    if not sources:
        sys.exit('至少给 --real 之一')

    all_pairs, n_orphan = [], 0
    for src in sources:
        pairs, orphans = scan(src)
        all_pairs += pairs
        n_orphan += orphans
    n_pos = sum(1 for _, lbl, _ in all_pairs
                if lbl and os.path.getsize(lbl) > 0)
    print(f'来源：{len(all_pairs)} 张（正样本 {n_pos}，空/无标签背景 {len(all_pairs) - n_pos}，'
          f'无 txt 补空 {n_orphan}）')

    train, val = split_pairs(all_pairs, args.val_frac, args.seed,
                             pin_train=set(args.pin_train), pin_val=set(args.val_fragments))
    if not any(lbl and os.path.getsize(lbl) > 0 for _, lbl, _ in val):
        sys.exit('val 里一张正样本都没有——用 --val-fragments 钉一个正样本段')
    if set(args.pin_train) & set(args.val_fragments):
        sys.exit('同一片段同时钉 train 和 val——检查参数')

    for split in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)

    copied, empty_lbl = 0, 0
    for split, bucket in (('train', train), ('val', val)):
        for img, lbl, name in bucket:
            shutil.copy2(img, os.path.join(args.out, 'images', split, name))
            dst_lbl = os.path.join(args.out, 'labels', split,
                                   os.path.splitext(name)[0] + '.txt')
            if lbl is None:
                open(dst_lbl, 'w').close()
                empty_lbl += 1
            else:
                shutil.copy2(lbl, dst_lbl)
                bad = validate_label(dst_lbl)
                if bad:
                    sys.exit(f'坏标签 {lbl} 行 {bad}——修标后再建集')
            copied += 1

    import yaml
    with open(os.path.join(args.out, 'data.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({'names': {0: 'h_marker'}, 'train': 'images/train',
                        'val': 'images/val'}, f, allow_unicode=True)

    version = args.version or f'hmarker_v1_{datetime.now().strftime("%Y%m%d")}'
    with open(os.path.join(args.out, 'dataset_version.md'), 'w',
              encoding='utf-8') as f:
        f.write(f'# {version}\n\n'
                f'- 共 {len(all_pairs)} 张：正样本 {n_pos}，背景负样本 '
                f'{len(all_pairs) - n_pos}（无 txt 补空 {n_orphan}）\n'
                f'- 切分：按**片段** {args.val_frac:.0%}（连续帧不跨集，防验证泄题）\n'
                + (f'- 钉扎 val 片段: {", ".join(args.val_fragments)}｜'
                   f'钉扎 train 片段: {", ".join(args.pin_train)}\n'
                   if args.val_fragments or args.pin_train else '')
                + f'- train {len(train)} / val {len(val)}，总计 {copied}，'
                f'空标签 {empty_lbl}\n'
                f'- 类别：0 = h_marker（单类 det；框贴白圆外沿）\n'
                f'- 建集时间 {datetime.now().strftime("%Y-%m-%d %H:%M")}，'
                f'seed={args.seed}\n')
    print(f'完成: {args.out}（train {len(train)} / val {len(val)}）')


if __name__ == '__main__':
    main()
