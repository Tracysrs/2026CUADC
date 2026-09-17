#!/usr/bin/env python3
"""白筒分割合成数据渲染器（YOLO-seg 多边形标签，类 0 = barrel）。

为什么重做合成集（设计文档 §8.1）：v1 合成集（2220 张）只导出了**检测框**，
seg 模型要的是多边形掩码——本渲染器直接程序化出掩码（合成数据掩码免费），
并复刻比赛成像几何：蓝地/水泥地背景、三档筒径 15/20/25cm、1.5~4.5m 高度、
轻微斜视椭圆、软边、阴影、光照梯度、亮度干扰物（教模型拒收非筒白块）。

用法：
  python render_barrel_synthetic.py --out synth_barrel --count 2000 --seed 7
产出：
  synth_barrel/images/train|val/*.jpg  +  labels/train|val/*.txt（YOLO-seg）
"""

import argparse
import math
import os
import random

import cv2
import numpy as np

DIAMS = (0.15, 0.20, 0.25)          # 1/2/3 号筒口直径（m）
FX_RANGE = (900.0, 1300.0)          # BL-500W-335 1080p 附近采样
H_RANGE = (1.5, 4.5)                # 相机离地高度（m）

# 蓝地（有效区）与水泥地（无效区）基色 BGR 采样域
BLUE_GROUND = ((150, 70, 30), (215, 110, 50))
GRAY_GROUND = ((110, 110, 110), (180, 180, 180))


def hex_polygon(cx, cy, a, b, rot_deg, n=24):
    """椭圆 → n 点多边形（YOLO-seg 标签用，归一化在调用侧做）。"""
    pts = []
    for i in range(n):
        th = 2 * math.pi * i / n
        x = a / 2 * math.cos(th)
        y = b / 2 * math.sin(th)
        c, s = math.cos(math.radians(rot_deg)), math.sin(math.radians(rot_deg))
        pts.append((cx + x * c - y * s, cy + x * s + y * c))
    return pts


def render_one(rng, size):
    w, h = size
    # ---- 背景：蓝地/水泥地 + 光照梯度 + 污渍 + 噪声 ----
    (b0, b1) = BLUE_GROUND if rng.random() < 0.6 else GRAY_GROUND
    base = rng.uniform(b0, b1)
    img = np.empty((h, w, 3), np.float32)
    img[:, :] = base
    gx = rng.uniform(-1, 1)
    grad = np.linspace(-22, 22, w, dtype=np.float32)[None, :, None] * gx
    gy = rng.uniform(-1, 1)
    grad = grad + np.linspace(-18, 18, h, dtype=np.float32)[:, None, None] * gy
    img += grad
    for _ in range(rng.integers(2, 7)):          # 地面污渍/暗斑
        ex, ey = rng.integers(0, w), rng.integers(0, h)
        ea, eb = rng.integers(w // 12, w // 4), rng.integers(h // 12, h // 4)
        shade = rng.uniform(-28, 14)
        cv2.ellipse(img, (int(ex), int(ey)), (int(ea), int(eb)),
                    rng.uniform(0, 180), 0, 360, (base[0] + shade,
                    base[1] + shade, base[2] + shade), -1)
    img += rng.normal(0, 3.0, img.shape)

    labels = []
    n_bucket = int(rng.choice([1, 2, 2, 3, 3, 3]))
    placed = []
    for _ in range(n_bucket):
        for _try in range(20):
            diam = float(rng.choice(DIAMS))
            hh = rng.uniform(*H_RANGE)
            fx = rng.uniform(*FX_RANGE)
            diam_px = diam * fx / hh
            if diam_px < 14 or diam_px > min(w, h) * 0.55:
                continue
            a = diam_px
            b = diam_px * rng.uniform(0.86, 1.0)   # 轻微斜视 → 短轴压缩
            rot = rng.uniform(0, 180)
            cx = rng.uniform(a / 2 + 8, w - a / 2 - 8)
            cy = rng.uniform(b / 2 + 8, h - b / 2 - 8)
            if all(math.hypot(cx - px, cy - py) > (a + pa) / 2 * 1.25
                   for px, py, pa, _ in placed):
                placed.append((cx, cy, a, b))
                # 阴影（筒侧光）：向一侧的暗弧
                sh = rng.uniform(10, 26)
                cv2.ellipse(img, (int(cx + rng.uniform(-a / 4, a / 4)),
                                  int(cy + b / 4)),
                            (int(a / 2), int(b / 2 * 0.8)), rot, 0, 360,
                            (base[0] - sh, base[1] - sh, base[2] - sh), -1)
                # 筒口白面（软边 = 高斯模糊由整体降噪后处理近似）
                cv2.ellipse(img, (int(cx), int(cy)), (int(a / 2), int(b / 2)),
                            rot, 0, 360, (245, 245, 245), -1)
                # 筒口内壁暗环（薄），模拟立体感
                cv2.ellipse(img, (int(cx), int(cy)),
                            (int(a / 2), int(b / 2)), rot, 0, 360,
                            (120, 120, 120), max(1, int(a * 0.03)))
                labels.append((cx, cy, a, b, rot))
                break
    # ---- 干扰物：亮白非筒块（地面标线/反光），教模型拒收 ----
    for _ in range(int(rng.integers(0, 3))):
        bw, bh = int(rng.integers(w // 20, w // 6)), int(rng.integers(3, 10))
        bx, by = int(rng.integers(0, w - bw)), int(rng.integers(0, h - bh))
        bright = rng.uniform(190, 235)
        cv2.rectangle(img, (bx, by), (bx + bw, by + bh),
                      (bright, bright, bright), -1)

    img = np.clip(img, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:                        # 轻微整体模糊（MJPG/运动近似）
        img = cv2.GaussianBlur(img, (3, 3), 0)
    return img, labels


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='synth_barrel')
    ap.add_argument('--count', type=int, default=2000)
    ap.add_argument('--val-frac', type=float, default=0.05)
    ap.add_argument('--size', default='1280x720')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    pyrng = random.Random(args.seed)
    w, h = (int(v) for v in args.size.lower().split('x'))
    for split in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)

    n_val = max(1, int(args.count * args.val_frac))
    for i in range(args.count):
        split = 'val' if i < n_val else 'train'
        img, labels = render_one(rng, (w, h))
        # 命名无下划线 → build_barrel_dataset 的片段切分把每帧视为独立片段（合成帧随机切）
        name = f'synth{i:05d}.jpg'
        cv2.imwrite(os.path.join(args.out, 'images', split, name), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        lines = []
        for (cx, cy, a, b, rot) in labels:
            pts = hex_polygon(cx, cy, a, b, rot)
            norm = ' '.join(f'{min(max(x / w, 0), 1):.5f} {min(max(y / h, 0), 1):.5f}'
                            for x, y in pts)
            lines.append(f'0 {norm}')
        with open(os.path.join(args.out, 'labels', split,
                               name.replace('.jpg', '.txt')), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        if (i + 1) % 200 == 0:
            print(f'{i + 1}/{args.count}')

    import yaml
    with open(os.path.join(args.out, 'data.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({'names': {0: 'barrel'}, 'train': 'images/train',
                        'val': 'images/val'}, f, allow_unicode=True)
    print(f'完成: {args.out}（train {args.count - n_val} / val {n_val}；'
          f'注意 data.yaml 不写 path 键——仓库铁律）')


if __name__ == '__main__':
    main()
