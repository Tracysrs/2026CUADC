#!/usr/bin/env python3
"""相机内参标定工具（BL-500W-335，棋盘格法 + 已知尺寸快速校验）。

用途（设计文档 §7）：真机内参从未标定前，感知节点按 datasheet FOV 推内参
（fx 精度受限，直径/位置误差被直径先验部分兜住但不达标）——外场前必须完成本标定。

用法（Jetson 或任何装了 opencv-python 的机器）：
  # 1) 棋盘格标定：打印 9x6 内角点棋盘（方格边长已知），对相机摆 15~25 个
  #    姿态（远近/倾角/九宫格位置都要覆盖），存入目录后：
  python3 calibrate_camera.py calibrate --dir ~/calib_captures \
      --pattern 9x6 --square 0.025 --out ~/cuadc_models/camera_calib.yaml

  # 2) 已知尺寸快速校验（免棋盘，悬停/手持相机离靶已知高度）：
  #    拍一张白圆（如 ø15cm 筒口）在画面中心，告诉脚本真实直径与相机高度：
  python3 calibrate_camera.py check --image ~/snap.jpg --diam 0.15 --height 2.0 \
      [--calib ~/cuadc_models/camera_calib.yaml]

标定后：把 yaml 放到节点参数 calib_path 指的路径，两节点重启即生效；
比赛模式把 bucket_perception_params.yaml 的 allow_uncalibrated 改 false。
"""

import argparse
import glob
import math
import os
import sys
from datetime import datetime

import cv2
import numpy as np

BL_HFOV_RAD = math.radians(81.8)
BL_VFOV_RAD = math.radians(66.0)


def cmd_calibrate(args):
    paths = sorted(sum([glob.glob(os.path.join(args.dir, f'*{ext}'))
                        for ext in ('.jpg', '.png', '.jpeg', '.bmp')], []))
    if len(paths) < 10:
        sys.exit(f'棋盘图不足：{len(paths)} 张（{args.dir}），至少 10 张、建议 15~25 张')

    cols, rows = (int(v) for v in args.pattern.lower().split('x'))
    pattern = (cols, rows)                     # 内角点数
    square = args.square
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    obj_points, img_points = [], []
    size = None
    used = 0
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f'跳过不可读: {p}')
            continue
        size = (img.shape[1], img.shape[0])
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print(f'未检出棋盘: {os.path.basename(p)}')
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        obj_points.append(objp)
        img_points.append(corners)
        used += 1

    if used < 10:
        sys.exit(f'有效棋盘图仅 {used} 张，标定不可靠，重新采集（姿态要多样）')

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, size, None, None)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    per_view = [float(np.linalg.norm(
        (cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)[0]
         - img_points[i]).reshape(-1, 2), axis=1).mean())
        for i in range(used)]
    worst = max(per_view)

    print(f'\n标定完成：{used} 张, RMS={rms:.3f}px, 最差单图={worst:.3f}px')
    print(f'fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}')
    print(f'dist={np.round(dist.ravel(), 6).tolist()}')
    print(f'fy/fx={fy / fx:.3f}（16:9 裁剪合理值 ~0.75，偏离大=标定差）')
    if rms > 1.0:
        print('⚠️ RMS > 1.0px：棋盘平整度/对焦/姿态多样性不足，建议重拍重标')

    # 与 datasheet FOV 交叉核对
    fx_hfov = (size[0] / 2.0) / math.tan(BL_HFOV_RAD / 2.0)
    print(f'datasheet HFOV 推 fx≈{fx_hfov:.0f}，标定 fx={fx:.0f}，'
          f'偏差 {(fx - fx_hfov) / fx_hfov * 100:+.1f}%（>10% 记录进坑清单）')

    import yaml
    out = dict(
        camera='BL-500W-335',
        source='calib',
        method=f'chessboard {args.pattern} square={square}',
        image_width=size[0], image_height=size[1],
        fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
        dist_coeffs=[float(v) for v in np.asarray(dist).ravel()],
        reproj_error_px=float(rms),
        frames_used=int(used),
        calibrated_at=datetime.now().strftime('%Y-%m-%d %H:%M'),
    )
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
    print(f'已写出 {args.out} —— 供 bucket_perception / h_circle 节点 calib_path 加载')


def cmd_check(args):
    img = cv2.imread(args.image)
    if img is None:
        sys.exit(f'图片不可读: {args.image}')
    h, w = img.shape[:2]
    frame = img
    dets = _white_circle_candidates(frame)
    if not dets:
        sys.exit('未检出白色圆目标——用白圆靶（或筒口）置于画面中心重拍')
    u, v, a, b = max(dets, key=lambda d: d[2])
    diam_px = (a + b) / 2.0
    fx_est = diam_px * args.height / args.diam

    print(f'检出圆心 ({u:.0f},{v:.0f})（画面中心应为 ({w // 2},{h // 2})），'
          f'直径 {diam_px:.1f}px')
    print(f'已知：直径 {args.diam}m @ 高度 {args.height}m → fx≈{fx_est:.0f}')
    print(f'16:9 datasheet 推 fx≈{(w / 2) / math.tan(BL_HFOV_RAD / 2):.0f}')
    if args.calib and os.path.isfile(args.calib):
        import yaml
        with open(args.calib, 'r', encoding='utf-8') as f:
            c = yaml.safe_load(f)
        rel = (fx_est - c['fx']) / c['fx'] * 100
        print(f'标定文件 fx={c["fx"]:.0f} → 与本次快检偏差 {rel:+.1f}%'
              f'{"（>5% 复查标定/高度读数）" if abs(rel) > 5 else " OK"}')
        if abs(v - h / 2) > 0.1 * h or abs(u - w / 2) > 0.1 * w:
            print('⚠️ 靶不在画面中心 10% 内：离心位置受畸变影响，快检结论仅供参考')
    else:
        print('（未提供 --calib 或文件不存在：仅输出快检 fx）')


def _white_circle_candidates(bgr):
    """简易白圆候选（快检专用，非生产检测链）。"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0]
    A = lab[:, :, 1].astype(np.int16)
    B = lab[:, :, 2].astype(np.int16)
    thr = max(150, float(np.median(L)) + 20)
    mask = ((L > thr) & (np.abs(A - 128) < 25) & (np.abs(B - 128) < 25)
            ).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in contours:
        if cv2.contourArea(cnt) < 400 or len(cnt) < 5:
            continue
        (eu, ev), (d_a, d_b), _ = cv2.fitEllipse(cnt)
        if min(d_a, d_b) / max(d_a, d_b) < 0.7:
            continue
        out.append((eu, ev, d_a, d_b))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    c1 = sub.add_parser('calibrate', help='棋盘格标定 → calib.yaml')
    c1.add_argument('--dir', required=True, help='棋盘图目录')
    c1.add_argument('--pattern', default='9x6', help='内角点数，如 9x6')
    c1.add_argument('--square', type=float, default=0.025, help='方格边长(m)')
    c1.add_argument('--out', default='camera_calib.yaml', help='输出 yaml 路径')
    c1.set_defaults(func=cmd_calibrate)

    c2 = sub.add_parser('check', help='已知尺寸靶快速校验 fx')
    c2.add_argument('--image', required=True)
    c2.add_argument('--diam', type=float, required=True, help='靶真实直径(m)')
    c2.add_argument('--height', type=float, required=True, help='相机离靶高度(m)')
    c2.add_argument('--calib', default='', help='标定 yaml（可选，做交叉核对）')
    c2.set_defaults(func=cmd_check)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
