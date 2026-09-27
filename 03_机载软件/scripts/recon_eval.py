#!/usr/bin/env python3
"""侦察判读离线评估工具（2026-09-27）：移动/模糊场景下融合规则的准入闸门。

背景：SSOT §4.4 判据（票数≥5、置信中位≥0.8、票数≥3×次高）在移动场景会
大量留空（运动模糊拖低单帧置信 + 快速横移拆碎关联票）。候选改动
（conf_agg='best_k'、assoc_predict=True，2026-09-27 已在 recon_fusion 落码、
默认关）必须先过本工具的数据关：**留空率显著降、错填率保持 ≈0** 才准入
（计分语义：留空 0 分、错填 -100）。

三段式（检测与评估解耦——评估段纯标准库，任何机器可跑）：
  ① genseq  生成带 GT 的合成连续序列（复用 gen_synthetic 渲染与运动模糊增广，
            模糊量按真机物理换算 v×曝光×FX÷高度）
  ② detect  ultralytics 跑序列 → 检测缓存 cache.jsonl（复刻节点完整链：
            FrameGate 门控 + 灰区 ROI 放大重推，缓存的是融合入口的过门控检测；
            需模型环境，训练机/Jetson 跑）
  ③ replay  纯 Python 回放：检测流 × 规则组合（median/best_k × 外推开/关）
            × min_median_conf 扫描 → 确认/留空/错填矩阵（markdown 报告）
            无模型环境时 --simulate 用玩具检测器冒烟（数字不作为准入依据）

用法:
  python scripts/recon_eval.py genseq --out ../_reconeval/seq1 --windows 32
  python scripts/recon_eval.py detect --model best.pt --seq ../_reconeval/seq1
  python scripts/recon_eval.py replay --seq ../_reconeval/seq1
  python scripts/recon_eval.py replay --seq ../_reconeval/seq1 --simulate

口径注意:
  - 缓存的是"过门控后"检测，故 reject_top1/灰区门槛不可在 replay 段重扫
    （要扫它们须改 detect 段重跑）；replay 段扫的是融合层三参数。
  - 序列 GT = 每窗单标识（合成保证），匹配取与 GT 末帧框 IoU 最大的累积器。
"""

import argparse
import importlib.util
import json
import math
import pathlib
import random
import sys
from statistics import median

HERE = pathlib.Path(__file__).resolve().parent   # 03_机载软件/scripts
PKG_ROOT = HERE.parent / 'cuadc_perception'      # 03_机载软件/cuadc_perception（包外层）
sys.path.insert(0, str(PKG_ROOT))

from cuadc_perception.recon_fusion import (   # noqa: E402
    Detection, FrameGate, MarkerFusion, bbox_iou,
)

REPO = PKG_ROOT.parents[1]                    # 仓库根（cuadc_perception→03_机载软件→根）
HAZARD_DIR = (REPO / '01_视觉感知' / '数据与训练' / 'v2训练流水线' /
              'hazard_labels_v2_11class' / 'hazard_labels')

# 节点同款常量（改节点参数时此处同步）
ASSOC_IOU = 0.3
VOTE_RATIO = 3.0
COMP_IOU = 0.45          # runner-up 竞争框 IoU 门限（节点 _detect 同款）
BOOST_SCALE = 2.0        # 灰区 ROI 放大倍数
MAX_BOOST = 4            # 每帧灰区重推上限
NEG_CLASS = 10           # barrel 类：不计错填（侦察判读对象是 0~9 标识）
BLANK = -1

BUCKETS = [(0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 99.0)]
COMBOS = [  # (名称, conf_agg, assoc_predict)
    ('median(现行)', 'median', False),
    ('best_k', 'best_k', False),
    ('外推', 'median', True),
    ('best_k+外推', 'best_k', True),
]


def _load_gen():
    spec = importlib.util.spec_from_file_location(
        'gen_synthetic', HAZARD_DIR / 'gen_synthetic.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- genseq ----

def cmd_genseq(a):
    import cv2
    import numpy as np
    g = _load_gen()
    rng = np.random.default_rng(a.seed)
    seeds = g.load_seeds(str(HAZARD_DIR / 'seeds'))
    out = pathlib.Path(a.out)
    (out / 'frames').mkdir(parents=True, exist_ok=True)

    dt = 1.0 / a.fps
    windows = []
    for w in range(a.windows):
        neg = rng.random() < a.neg_frac
        if neg:
            v, blur, h_m, drift = 0.0, 0.0, 2.5, 0.0
            frames_meta = []
            for j in range(a.frames):
                img = g.degrade(g.make_background(rng), rng, h_m=h_m)
                name = f'w{w:03d}_f{j:03d}.jpg'
                g._imwrite_u(str(out / 'frames' / name), img,
                             [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_q])
                frames_meta.append({'file': name, 'bbox': None, 'cls': None})
            windows.append(dict(win=w, neg=True, cls=None, h_m=h_m, v_ms=v,
                                blur_px=blur, drift_deg=drift,
                                frames=frames_meta))
            continue

        cls = int(rng.integers(0, 10))
        h_m = float(rng.uniform(a.h_low, a.h_high))
        v = float(rng.uniform(0.0, a.v_max))
        drift = float(rng.uniform(0.0, 360.0))
        ang = float(rng.uniform(0, 360))
        tilt = float(rng.uniform(0, 0.25))
        r_px = g.BARREL_M / h_m * g.FX / 2
        s_px = g.LABEL_M / h_m * g.FX
        blur = v * g.MOTION_EXPOSURE_S * g.FX / h_m
        step = v * dt * g.FX / h_m
        rad = math.radians(drift)
        sx, sy = math.cos(rad), math.sin(rad)
        # 视场半宽做轨迹中心采样界：允许轨迹穿越/部分出画（真实连续窗剖面：
        # 高速时标识只在窗内停留一段，其余帧=纯背景），不强求全程居中
        half_travel = step * (a.frames - 1) / 2
        margin = s_px / 2 + blur + 6

        # 至少 1/4 帧可见才成窗（否则重摇轨迹中心），保证窗口有可判读目标
        for _ in range(20):
            cx = rng.uniform(max(0.0, margin - half_travel),
                             min(g.W, g.W - margin + half_travel))
            cy = rng.uniform(max(0.0, margin - half_travel),
                             min(g.H, g.H - margin + half_travel))
            n_vis = sum(
                margin <= cx + sx * (f - (a.frames - 1) / 2) * step <= g.W - margin
                and margin <= cy + sy * (f - (a.frames - 1) / 2) * step <= g.H - margin
                for f in range(a.frames))
            if n_vis >= max(3, a.frames // 4):
                break

        frames_meta = []
        for j in range(a.frames):
            px = cx + sx * (j - (a.frames - 1) / 2) * step
            py = cy + sy * (j - (a.frames - 1) / 2) * step
            in_frame = (margin <= px <= g.W - margin
                        and margin <= py <= g.H - margin)
            bg = g.make_background(rng)
            bbox = None
            if in_frame:
                g.draw_barrel(bg, (int(px), int(py)), r_px, rng)
                bbox, _ = g.paste_label(bg, seeds[cls], (px, py), s_px, rng,
                                        ang=ang, tilt=tilt)
            img = g.degrade(bg, rng, h_m=h_m, motion_speed=v, motion_angle=drift)
            name = f'w{w:03d}_f{j:03d}.jpg'
            g._imwrite_u(str(out / 'frames' / name), img,
                         [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_q])
            if bbox is None:
                frames_meta.append({'file': name, 'bbox': None, 'cls': None})
            else:
                x0, y0, _, s = bbox
                frames_meta.append({'file': name,
                                    'bbox': [x0 + s / 2, y0 + s / 2, s, s],
                                    'cls': cls})
        windows.append(dict(win=w, neg=False, cls=cls, h_m=h_m, v_ms=v,
                            blur_px=blur, drift_deg=drift, frames=frames_meta))

    gt = dict(fps=a.fps, frames_per_window=a.frames,
              classes=g.CLASSES + ['barrel'], windows=windows)
    (out / 'gt.json').write_text(json.dumps(gt, ensure_ascii=False, indent=1),
                                 encoding='utf-8')
    n_pos = sum(1 for x in windows if not x['neg'])
    print(f'genseq 完成: {a.windows} 窗（正 {n_pos}/负 {a.windows - n_pos}）'
          f'× {a.frames} 帧 @ {a.fps}Hz，v∈[0,{a.v_max}]m/s → {out}/')


# ---------------------------------------------------------------- detect ----

def _boost_crop(img, det):
    """节点 _boost 同款：ROI 裁剪放大重推，坐标映射回原图（hazard_recon_node）。"""
    import cv2
    H, W = img.shape[:2]
    hw = max(det['w'], 32.0) * 0.75
    hh = max(det['h'], 32.0) * 0.75
    x0, y0 = max(0, int(det['u'] - hw)), max(0, int(det['v'] - hh))
    x1, y1 = min(W, int(det['u'] + hw)), min(H, int(det['v'] + hh))
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        return None
    big = cv2.resize(crop, None, fx=BOOST_SCALE, fy=BOOST_SCALE,
                     interpolation=cv2.INTER_CUBIC)
    return big, x0, y0


def cmd_detect(a):
    import cv2
    import numpy as np
    from ultralytics import YOLO
    gt = json.loads((pathlib.Path(a.seq) / 'gt.json').read_text(encoding='utf-8'))
    cache_path = pathlib.Path(a.seq) / 'cache.jsonl'
    if cache_path.exists() and not a.force:
        sys.exit(f'{cache_path} 已存在（--force 覆盖）')

    model = YOLO(a.model, task='detect')
    black = np.zeros((a.imgsz, a.imgsz, 3), dtype=np.uint8)
    for _ in range(3):
        model.predict(black, imgsz=a.imgsz, verbose=False)

    gate = FrameGate()
    n_lines = 0
    with open(cache_path, 'w', encoding='utf-8') as f:
        for win in gt['windows']:
            for j, fm in enumerate(win['frames']):
                img = cv2.imdecode(
                    np.fromfile(str(pathlib.Path(a.seq) / 'frames' / fm['file']),
                                dtype=np.uint8), cv2.IMREAD_COLOR)
                r = model.predict(img, imgsz=a.imgsz, conf=a.conf,
                                  iou=a.iou, verbose=False)[0]
                boxes = r.boxes
                rows = [tuple(float(v) for v in row)
                        for row in boxes.xywhn.cpu().numpy()]
                confs = [float(v) for v in boxes.conf.cpu().numpy()]
                clss = [int(v) for v in boxes.cls.cpu().numpy()]
                H, W = img.shape[:2]
                dets = []
                for i, (row, conf, cls) in enumerate(zip(rows, confs, clss)):
                    cx, cy, w, h = row
                    runner = 0.0
                    for k, (r2, c2) in enumerate(zip(rows, clss)):
                        if k == i or c2 == cls:
                            continue
                        if _iou_xywhn(row, r2) >= COMP_IOU:
                            runner = max(runner, confs[k])
                    dets.append(dict(u=cx * W, v=cy * H, w=w * W, h=h * H,
                                     cls=cls, conf=conf, runner=runner))
                accepted = []
                gray = []
                for d in dets:
                    gres = gate.check(_mk_det(d, 0.0))
                    if gres.accepted:
                        accepted.append(d)
                    elif gres.need_boost:
                        gray.append(d)
                gray.sort(key=lambda d: d['conf'], reverse=True)
                for d in gray[:MAX_BOOST]:
                    bb = _boost_crop(img, d)
                    if bb is None:
                        continue
                    big, x0, y0 = bb
                    rb = model.predict(big, imgsz=a.imgsz, conf=a.conf,
                                       iou=a.iou, verbose=False)[0]
                    if len(rb.boxes) == 0:
                        continue
                    bi = int(rb.boxes.conf.argmax())
                    bcx, bcy, bw, bh = (float(v) for v in rb.boxes.xywhn[bi])
                    BW, BH = big.shape[1], big.shape[0]
                    boosted = dict(u=x0 + bcx * BW / BOOST_SCALE,
                                   v=y0 + bcy * BH / BOOST_SCALE,
                                   w=bw * BW / BOOST_SCALE,
                                   h=bh * BH / BOOST_SCALE,
                                   cls=int(rb.boxes.cls[bi]),
                                   conf=float(rb.boxes.conf[bi]), runner=0.0)
                    if gate.check_boosted(_mk_det(boosted, 0.0)).accepted:
                        accepted.append(boosted)
                f.write(json.dumps(dict(
                    win=win['win'], frame=j, t=j / gt['fps'],
                    accepted=[{k: round(v, 2) for k, v in d.items()}
                              for d in accepted])) + '\n')
                n_lines += 1
    print(f'detect 完成: {n_lines} 帧 → {cache_path}')


def _mk_det(d, t):
    return Detection(u=d['u'], v=d['v'], w=d['w'], h=d['h'],
                     class_id=d['cls'], confidence=d['conf'],
                     runner_up_conf=d['runner'], stamp_s=t)


def _iou_xywhn(a, b):
    ax1, ay1, ax2, ay2 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1, bx2, by2 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    iw, ih = min(ax2, bx2) - max(ax1, bx1), min(ay2, by2) - max(ay1, by1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


# ---------------------------------------------------------------- replay ----

def _simulate_cache(gt, seed):
    """玩具检测器（仅验 harness 机制，数字不作为准入依据）：
    置信 μ=0.93·exp(-blur/14)——校准到台架定性实况：静态中位 ~0.9 可确认，
    1m/s(≈5px) 边缘、2m/s+ 死区；模糊越高丢帧越多、低置信加类混淆。"""
    rng = random.Random(seed)
    lines = []
    for win in gt['windows']:
        for j, fm in enumerate(win['frames']):
            acc = []
            if not win['neg'] and fm['bbox']:
                mu = 0.93 * math.exp(-win['blur_px'] / 14.0)
                if rng.random() > 0.05 + 0.4 * max(0.0, mu - 0.45):
                    conf = max(0.03, min(0.98, rng.gauss(mu, 0.06)))
                    cls = win['cls']
                    runner = conf * rng.uniform(0.1, 0.6)
                    if conf < 0.75 and rng.random() < 0.35:
                        cls = rng.choice([c for c in range(10) if c != win['cls']])
                    x, y, w, h = fm['bbox']
                    jit = 3 + win['blur_px']
                    acc = [dict(u=x + rng.uniform(-jit, jit),
                                v=y + rng.uniform(-jit, jit),
                                w=w + rng.uniform(-jit, jit),
                                h=h + rng.uniform(-jit, jit),
                                cls=cls, conf=round(conf, 3),
                                runner=round(runner, 3))]
            lines.append(dict(win=win['win'], frame=j, t=j / gt['fps'],
                              accepted=acc))
    return lines


def _verdict_stats(gt, frames_by_win, fusion_kw):
    """一套融合参数跑全部窗口，返回逐窗结果 [dict(win,neg,result,matched)]."""
    out = []
    for win in gt['windows']:
        fusion = MarkerFusion(assoc_iou=ASSOC_IOU, vote_ratio=VOTE_RATIO,
                              **fusion_kw)
        for fr in frames_by_win[win['win']]:
            fusion.update([_mk_det(d, fr['t']) for d in fr['accepted']])
        verdicts = fusion.verdicts()
        rec = dict(win=win['win'], neg=win['neg'], result=None, fp_extra=False)
        if win['neg']:
            rec['result'] = ('fp' if any(BLANK < v.class_id != NEG_CLASS
                                         for v in verdicts) else 'tn')
            out.append(rec)
            continue
        bbox = win['frames'][-1]['bbox']
        best_iou, best_i = 0.3, None
        vis = [(j, fm) for j, fm in enumerate(win['frames']) if fm['bbox']]
        for i, v in enumerate(verdicts):
            acc = fusion.markers[i]
            # 高速窗标识会中途出画：用累积器末次更新时刻对齐最近的可见 GT 帧
            _, fm = min(vis, key=lambda jf: abs(jf[0] / gt['fps']
                                                - acc.last_stamp_s))
            bb = fm['bbox']
            iou = bbox_iou(bb[0], bb[1], bb[2], bb[3],
                           acc.last_u, acc.last_v, acc.last_w, acc.last_h)
            if iou > best_iou:
                best_iou, best_i = iou, i
        matched = verdicts[best_i] if best_i is not None else None
        if matched is None:
            rec['result'] = 'miss'           # GT 标识连累积器都没建起来
        elif matched.class_id == BLANK:
            rec['result'] = 'blank'
        elif matched.class_id == win['cls']:
            rec['result'] = 'correct'
        else:
            rec['result'] = 'wrong'
        rec['fp_extra'] = any(
            i != best_i and BLANK < v.class_id != NEG_CLASS
            for i, v in enumerate(verdicts))
        out.append(rec)
    return out


def cmd_replay(a):
    seq = pathlib.Path(a.seq)
    gt = json.loads((seq / 'gt.json').read_text(encoding='utf-8'))
    if a.simulate:
        cache = _simulate_cache(gt, a.seed)
        src = f'模拟检测器(--simulate，玩具口径，仅验机制)'
    else:
        cache_path = seq / 'cache.jsonl'
        if not cache_path.exists():
            sys.exit(f'缺 {cache_path}——先跑 detect 子命令（或 --simulate 冒烟）')
        cache = [json.loads(ln) for ln in
                 cache_path.read_text(encoding='utf-8').splitlines() if ln.strip()]
        src = str(cache_path)
    frames_by_win = {}
    for fr in cache:
        frames_by_win.setdefault(fr['win'], []).append(fr)

    conf_grid = [float(x) for x in a.conf_grid.split(',')]
    lines = [f'# recon_eval 回放报告', '',
             f'- 序列: `{seq}`（{len(gt["windows"])} 窗 × {gt["frames_per_window"]} 帧'
             f' @ {gt["fps"]}Hz，检测源: {src}）',
             f'- 计分口径: 留空 0 / 错填 -100（SSOT §4.4）；得分 = 确认数 − 100×错填数',
             f'- min_frames={a.min_frames} vote_ratio={a.vote_ratio} '
             f'assoc_iou={ASSOC_IOU}', '']

    # 表1：默认置信门槛下的规则组合对比
    lines += ['## 规则组合对比（min_median_conf=0.8）', '',
              '| 组合 | 确认率 | 留空率 | 错填率 | 误报窗 | 得分 |',
              '|---|---|---|---|---|---|']
    table1 = {}
    for name, agg, pred in COMBOS:
        kw = dict(min_frames=a.min_frames, min_median_conf=0.8,
                  conf_agg=agg, assoc_predict=pred)
        stats = _verdict_stats(gt, frames_by_win, kw)
        pos = [s for s in stats if not s['neg']]
        n = len(pos)
        cor = sum(1 for s in pos if s['result'] == 'correct')
        blk = sum(1 for s in pos if s['result'] in ('blank', 'miss'))
        wrg = sum(1 for s in pos if s['result'] == 'wrong')
        fp = sum(1 for s in stats if s['result'] == 'fp')
        score = cor - 100 * wrg
        table1[(name)] = stats
        lines.append(f'| {name} | {cor}/{n}={cor / n:.0%} | {blk / n:.0%} '
                     f'| {wrg / n:.0%} | {fp} | **{score}** |')
    lines.append('')

    # 表2：置信门槛扫描（4 组合 × 门槛 → 确/空/错）
    lines += ['## min_median_conf 扫描（格: 确认/留空/错填，正窗）', '',
              '| 门槛 | ' + ' | '.join(n for n, _, _ in COMBOS) + ' |',
              '|---|' + '---|' * len(COMBOS)]
    for mc in conf_grid:
        row = [f'{mc:.2f}']
        for name, agg, pred in COMBOS:
            kw = dict(min_frames=a.min_frames, min_median_conf=mc,
                      conf_agg=agg, assoc_predict=pred)
            stats = _verdict_stats(gt, frames_by_win, kw)
            pos = [s for s in stats if not s['neg']]
            n = len(pos)
            cor = sum(1 for s in pos if s['result'] == 'correct')
            blk = sum(1 for s in pos if s['result'] in ('blank', 'miss'))
            wrg = sum(1 for s in pos if s['result'] == 'wrong')
            row.append(f'{cor / n:.0%}/{blk / n:.0%}/{wrg / n:.0%}')
        lines.append('| ' + ' | '.join(row) + ' |')
    lines.append('')

    # 表3：模糊分桶（baseline vs best_k+外推 @0.8）
    lines += ['## 速度/模糊分桶（min_median_conf=0.8）', '',
              '| v (m/s) | 组合 | 确认 | 留空 | 错填 | 窗数 |',
              '|---|---|---|---|---|---|']
    stats_by_combo = {name: table1[name] for name, _, _ in COMBOS}
    gt_by_win = {w['win']: w for w in gt['windows']}
    for lo, hi in BUCKETS:
        wins = {w['win'] for w in gt['windows']
                if not w['neg'] and lo <= w['v_ms'] < hi}
        if not wins:
            continue
        for name, _, _ in COMBOS:
            pos = [s for s in stats_by_combo[name]
                   if s['win'] in wins]
            n = len(pos)
            cor = sum(1 for s in pos if s['result'] == 'correct')
            blk = sum(1 for s in pos if s['result'] in ('blank', 'miss'))
            wrg = sum(1 for s in pos if s['result'] == 'wrong')
            lines.append(f'| [{lo},{hi}) | {name} | {cor} | {blk} | {wrg} | {n} |')

    report = '\n'.join(lines) + '\n'
    print(report)
    out_path = seq / ('report_sim.md' if a.simulate else 'report.md')
    out_path.write_text(report, encoding='utf-8')
    print(f'报告已存 {out_path}')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)

    g = sub.add_parser('genseq', help='生成带 GT 合成序列')
    g.add_argument('--out', required=True)
    g.add_argument('--windows', type=int, default=32)
    g.add_argument('--frames', type=int, default=20)
    g.add_argument('--fps', type=float, default=5.0)
    g.add_argument('--seed', type=int, default=7)
    g.add_argument('--v-max', type=float, default=3.0)
    g.add_argument('--h-low', type=float, default=2.0)
    g.add_argument('--h-high', type=float, default=3.0)
    g.add_argument('--neg-frac', type=float, default=0.15)
    g.add_argument('--jpeg-q', type=int, default=92)
    g.set_defaults(fn=cmd_genseq)

    d = sub.add_parser('detect', help='ultralytics 跑序列出检测缓存')
    d.add_argument('--model', required=True)
    d.add_argument('--seq', required=True)
    d.add_argument('--imgsz', type=int, default=640)
    d.add_argument('--conf', type=float, default=0.10)
    d.add_argument('--iou', type=float, default=0.60)
    d.add_argument('--force', action='store_true')
    d.set_defaults(fn=cmd_detect)

    r = sub.add_parser('replay', help='纯 Python 回放扫掠（零第三方依赖）')
    r.add_argument('--seq', required=True)
    r.add_argument('--simulate', action='store_true',
                   help='无缓存时用玩具检测器冒烟（数字不作准入依据）')
    r.add_argument('--seed', type=int, default=11)
    r.add_argument('--conf-grid', default='0.6,0.7,0.75,0.8,0.85,0.9')
    r.add_argument('--min-frames', type=int, default=5)
    r.add_argument('--vote-ratio', type=float, default=3.0)
    r.set_defaults(fn=cmd_replay)

    a = p.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
