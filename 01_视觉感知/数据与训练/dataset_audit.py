#!/usr/bin/env python3
"""YOLO 数据集审计器（纯 stdlib）——SSOT §15-B4，规格来源 Re0 视觉 ch29。

用法（本机，数据集目录按 real_data 布局）：
    cd 01_视觉感知/数据与训练
    python dataset_audit.py --images real_data/images/train --labels real_data/labels/train --classes 11
    python dataset_audit.py ... --manifest manifest.csv   # image,label,split,session

审计项（只报告，绝不自动改标签——自动"修正"会掩盖坐标系错误）：
  1. 图片/标签互相缺失（按同名 stem 配对）；
  2. 标签逐行：非数值 / NaN/Inf / 类别越界 / 坐标越界 [0,1] / detect 零框
     (w 或 h ≤ 0) / seg 点数 < 3 或值个数为奇数；
  3. 精确重复图片（整文件 md5）；
  4. --manifest：session 跨 split 泄漏（同一 session 同时出现在训练与验证）。

退出码：0 = 无 error（warning 不计）；1 = 有 error（可接 CI）。
单元测试：test_dataset_audit.py（同目录，unittest 构造临时好/坏样本）。
"""

import argparse
import csv
import hashlib
import math
import sys
from collections import defaultdict
from pathlib import Path

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def parse_label_line(line: str, classes: int, task: str):
    """解析一行标签。返回 (cls, coords, error)。coords = 除 cls 外的浮点列表。"""
    parts = line.split()
    try:
        values = [float(x) for x in parts]
    except ValueError:
        return None, None, '非数值字段'
    if not values:
        return None, None, '空行（已跳过不应到这）'
    if not all(math.isfinite(v) for v in values):
        return None, None, '含 NaN/Inf'
    cls = int(values[0])
    if cls != values[0]:
        return None, None, '类别号非整数'
    if not 0 <= cls < classes:
        return None, None, f'类别越界 {cls} (classes={classes})'
    coords = values[1:]
    if task == 'detect':
        if len(coords) != 4:
            return None, None, f'detect 需要 4 坐标，得到 {len(coords)}'
        cx, cy, w, h = coords
        if not all(0.0 <= v <= 1.0 for v in (cx, cy, w, h)):
            return None, None, '坐标越界 [0,1]'
        if w <= 0.0 or h <= 0.0:
            return None, None, '零面积框 (w/h ≤ 0)'
        return cls, coords, None
    # segment: cls + 多边形 (x, y)*n
    if len(coords) % 2 != 0:
        return None, None, 'seg 坐标个数为奇数'
    if len(coords) < 6:
        return None, None, f'seg 多边形点数 {len(coords) // 2} < 3'
    if not all(0.0 <= v <= 1.0 for v in coords):
        return None, None, '多边形坐标越界 [0,1]'
    return cls, coords, None


def audit_dataset(images_dir: Path, labels_dir: Path, classes: int,
                  task: str = 'detect'):
    """审计一对 images/labels 目录。返回 (errors, warnings, stats)。
    errors/warnings 均为字符串列表；stats 供报告输出。"""
    errors, warnings = [], []
    if task not in ('detect', 'segment'):
        raise ValueError(f'未知 task: {task}')
    if classes < 1:
        raise ValueError('classes 必须 ≥ 1')

    image_files = sorted(p for p in images_dir.iterdir()
                         if p.suffix.lower() in IMAGE_EXTS)
    label_files = sorted(p for p in labels_dir.iterdir()
                         if p.suffix.lower() == '.txt')
    image_by_stem = {p.stem: p for p in image_files}
    label_by_stem = {p.stem: p for p in label_files}

    missing_labels = sorted(set(image_by_stem) - set(label_by_stem))
    missing_images = sorted(set(label_by_stem) - set(image_by_stem))
    for stem in missing_labels:
        errors.append(f'图片缺标签: {stem}')
    for stem in missing_images:
        errors.append(f'标签缺图片: {stem}')

    total_lines = 0
    total_boxes = 0
    for stem in sorted(set(label_by_stem) & set(image_by_stem)):
        path = label_by_stem[stem]
        text = path.read_text(encoding='utf-8', errors='replace')
        if not text.strip():
            # 空标签文件 = 有意的负样本（图内无目标），合法但记 warning 供人工确认
            warnings.append(f'空标签（负样本？）: {stem}')
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            if not line.strip():
                continue
            total_lines += 1
            cls, coords, err = parse_label_line(line, classes, task)
            if err is not None:
                errors.append(f'{path.name}:{lineno}: {err}  ← {line.strip()[:60]}')
                continue
            total_boxes += 1
            _ = cls, coords

    # 精确重复图片（整文件 md5；近似重复不在此工具范围）
    seen_md5 = {}
    for p in image_files:
        digest = hashlib.md5(p.read_bytes()).hexdigest()
        if digest in seen_md5:
            errors.append(f'精确重复图片: {p.name} 与 {seen_md5[digest]}')
        else:
            seen_md5[digest] = p.name

    stats = {
        'images': len(image_files),
        'labels': len(label_files),
        'paired': len(set(image_by_stem) & set(label_by_stem)),
        'label_lines': total_lines,
        'boxes': total_boxes,
        'duplicates': len(image_files) - len(seen_md5),
    }
    return errors, warnings, stats


def audit_manifest(manifest_path: Path):
    """校验 manifest（columns: image,label,split,session）。
    返回 (errors, session_split_map)。泄漏 = 同一 session 出现在多个 split。"""
    errors = []
    session_splits = defaultdict(set)
    with manifest_path.open(encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        required = {'image', 'label', 'split', 'session'}
        if not required.issubset(reader.fieldnames or ()):
            return [f'manifest 缺列，需要 {sorted(required)}，'
                    f'实际 {reader.fieldnames}'], session_splits
        for row in reader:
            session_splits[row['session']].add(row['split'])
    for session, splits in sorted(session_splits.items()):
        if len(splits) > 1:
            errors.append(f'session 泄漏: "{session}" 同时出现在 {sorted(splits)}')
    return errors, session_splits


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='YOLO 数据集审计器（只报告不改标签）')
    parser.add_argument('--images', required=True)
    parser.add_argument('--labels', required=True)
    parser.add_argument('--classes', type=int, required=True)
    parser.add_argument('--task', choices=('detect', 'segment'), default='detect')
    parser.add_argument('--manifest', help='CSV：image,label,split,session（防泄漏审计）')
    args = parser.parse_args(argv)

    images_dir, labels_dir = Path(args.images), Path(args.labels)
    for d in (images_dir, labels_dir):
        if not d.is_dir():
            print(f'ERROR: 目录不存在 {d}')
            return 1

    errors, warnings, stats = audit_dataset(
        images_dir, labels_dir, args.classes, args.task)

    if args.manifest:
        manifest_path = Path(args.manifest)
        if manifest_path.is_file():
            m_errors, session_splits = audit_manifest(manifest_path)
            errors.extend(m_errors)
            stats['sessions'] = len(session_splits)
        else:
            errors.append(f'manifest 不存在: {manifest_path}')

    print(f'=== 审计报告 ===')
    print(f'图片 {stats["images"]} / 标签 {stats["labels"]} / 配对 {stats["paired"]} '
          f'/ 标签行 {stats["label_lines"]} / 目标框 {stats["boxes"]} '
          f'/ 重复图片 {stats.get("duplicates", 0)}')
    if 'sessions' in stats:
        print(f'manifest session 数: {stats["sessions"]}')
    print(f'warnings: {len(warnings)}')
    for w in warnings[:20]:
        print(f'  [W] {w}')
    if len(warnings) > 20:
        print(f'  ... 其余 {len(warnings) - 20} 条省略')
    print(f'errors: {len(errors)}')
    for e in errors[:50]:
        print(f'  [E] {e}')
    if len(errors) > 50:
        print(f'  ... 其余 {len(errors) - 50} 条省略')

    print('RESULT:', 'PASS' if not errors else f'FAIL ({len(errors)} errors)')
    return 0 if not errors else 1


if __name__ == '__main__':
    sys.exit(main())
