# -*- coding: utf-8 -*-
"""CUADC 2026 危化标识检测 · v3 建集脚本(relabel 模式)。

mapped_dataset 成品集不在本机(30426 机器),经用户拍板走路 B:用现役 v2 best.onnx
对本机 train/(445 张,哈工大原始旧表标注)重打 10 类新表标签。依据:train/ 正是 v2
的训练域数据(训过 100ep,域内响应接近满分),2026-09-29 实测 99.0% 框 IoU≥0.5 对齐、
conf p50=0.906;混淆矩阵实锤旧表 10 类中 8 类系统性名实错位(完美双射),主映射:
  旧→新 = {0:0, 1:5, 2:9, 3:3, 4:1, 5:4, 6:7, 7:6, 8:8, 9:2}

标签生成规则(逐框):
  - 旧表框匹配上模型框(conf≥0.6)→ 用模型新表类别(等价"按图案重排"+模型客观裁决分歧)
  - 旧表框匹配上但 conf<0.6,或未匹配上 → 用主映射类别(结构性重排,不受该帧推理抖动影响)
  - 模型高置信(conf≥0.85)新增检出(旧表漏标)→ 收进 GT,修复负样本污染
  - 哈希命名 PNG(网页截图)剔除;无标注实拍帧→train 背景空标签(高置信检出除外)
切分纪律复刻 build_real_data.py(防连续帧泄题):clip 系列随机 10%(seed42)/ frame 系列
中段连续 10% / 1_ 系列末段 10% / val 只放有标注图。
直采组 aircraft_direct_20260929 并入:val 取每组开头连续帧(A1~A4 各3 + B1/B2 各4 = 20),
train 其余 100 张(组内连拍近静态的 caveat 见组 README,验收以台架 A/B 为准)。

用法:python build_v3_dataset.py   (产出 hazard_v3_direct/,train/ 只读不动)
"""
import json
import random
import re
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

random.seed(42)
HERE = Path(__file__).resolve().parent
SRC = HERE / "train"
DIRECT = HERE / "hazard_hgd_10cls_v2" / "aircraft_direct_20260929"
DST = HERE / "hazard_v3_direct"
ONNX = DIRECT / "best.onnx"
NAMES = ["爆炸品", "不燃气体", "刺激性", "放射性物品", "腐蚀品",
         "生物危害", "遇湿易燃物品", "有毒品", "自燃物品", "易燃"]
OLD2NEW = {0: 0, 1: 5, 2: 9, 3: 3, 4: 1, 5: 4, 6: 7, 7: 6, 8: 8, 9: 2}
CONF, IOU_NMS, IMGSZ, IOU_MATCH, HI_CONF = 0.25, 0.45, 640, 0.5, 0.85
VAL_PLAN = {"A1": 3, "A2": 3, "A3": 3, "A4": 3, "B1": 4, "B2": 4}

RE_CLIP = re.compile(r"^\d{4}_frame_\d+\.(jpg|png)$")
RE_V1 = re.compile(r"^1_\d+\.(jpg|png)$")


def series_of(name):
    if RE_CLIP.match(name):
        return "clip"
    if name.startswith("frame_"):
        return "frame"
    if RE_V1.match(name):
        return "v1"
    return "other"


def frame_no(name):
    m = re.search(r"(\d+)\.(jpg|png)$", name)
    return int(m.group(1)) if m else 0


def letterbox(img):
    h, w = img.shape[:2]
    r = min(IMGSZ / h, IMGSZ / w)
    nh, nw = round(h * r), round(w * r)
    canvas = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    top, left = (IMGSZ - nh) // 2, (IMGSZ - nw) // 2
    canvas[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return canvas, r, left, top


def nms(b, s, t):
    i = s.argsort()[::-1]
    keep = []
    while i.size:
        k = i[0]
        keep.append(k)
        if i.size == 1:
            break
        xx1 = np.maximum(b[k, 0], b[i[1:], 0]); yy1 = np.maximum(b[k, 1], b[i[1:], 1])
        xx2 = np.minimum(b[k, 2], b[i[1:], 2]); yy2 = np.minimum(b[k, 3], b[i[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        a1 = (b[k, 2] - b[k, 0]) * (b[k, 3] - b[k, 1])
        a2 = (b[i[1:], 2] - b[i[1:], 0]) * (b[i[1:], 3] - b[i[1:], 1])
        i = i[1:][inter / (a1 + a2 - inter + 1e-9) <= t]
    return keep


def main():
    sess = ort.InferenceSession(str(ONNX), providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0].name
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (DST / sub).mkdir(parents=True, exist_ok=True)

    report = {"boxes_model_cls": 0, "boxes_map_cls": 0, "boxes_unmatched_kept": 0,
              "boxes_added_hiconf": 0, "bg_train": 0}
    plan = {"train": [], "val": []}  # (img_path, new_label_rows)

    # ---------- 推理 + 重标 ----------
    imgs = sorted(p for p in (SRC / "images").iterdir() if series_of(p.name) != "other")
    print(f"有效图 {len(imgs)}(另有 {len(list((SRC/'images').iterdir()))-len(imgs)} 张哈希 PNG 剔除)")
    for imf in imgs:
        img = cv2.imdecode(np.frombuffer(imf.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        H, W = img.shape[:2]
        lp = SRC / "labels" / (imf.stem + ".txt")
        old_rows = []
        if lp.exists():
            old_rows = [l.split() for l in lp.read_text().strip().splitlines() if l.strip()]
        canvas, r, left, top = letterbox(img)
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, swapRB=True)
        out = sess.run(None, {inp: blob})[0][0]
        bx, cs = out[:4], out[4:]
        conf = cs.max(axis=0)
        idx = np.where(conf >= CONF)[0]
        cands = []
        for i in idx:
            cx, cy, bw, bh = bx[:, i]
            x1, y1 = (cx - bw / 2 - left) / r, (cy - bh / 2 - top) / r
            x2, y2 = (cx + bw / 2 - left) / r, (cy + bh / 2 - top) / r
            cands.append([max(x1, 0), max(y1, 0), min(x2, W), min(y2, H),
                          float(conf[i]), int(cs[:, i].argmax())])
        kept = [cands[i] for i in nms(np.array([c[:4] for c in cands]),
                                      np.array([c[4] for c in cands]), IOU_NMS)] if cands else []
        used = set()
        new_rows = []
        for row in old_rows:
            oc = int(row[0])
            ox, oy, ow, oh = map(float, row[1:])
            best, bi = 0.0, -1
            for j, (x1, y1, x2, y2, cf, cl) in enumerate(kept):
                xx1, yy1 = max((ox - ow / 2) * W, x1), max((oy - oh / 2) * H, y1)
                xx2, yy2 = min((ox + ow / 2) * W, x2), min((oy + oh / 2) * H, y2)
                inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
                iou = inter / (ow * W * oh * H + (x2 - x1) * (y2 - y1) - inter + 1e-9)
                if iou > best:
                    best, bi = iou, j
            if best >= IOU_MATCH and kept[bi][4] >= 0.6:
                cls = kept[bi][5]
                used.add(bi)
                report["boxes_model_cls"] += 1
            else:
                cls = OLD2NEW[oc]
                if best >= IOU_MATCH:
                    used.add(bi)
                    report["boxes_model_cls"] += 0  # conf<0.6:类别走主映射,检出不再复用
                report["boxes_map_cls"] += 1
            new_rows.append((cls, ox, oy, ow, oh))
        if old_rows:
            for j, (x1, y1, x2, y2, cf, cl) in enumerate(kept):
                if j not in used and cf >= HI_CONF:
                    new_rows.append((cl, (x1 + x2) / 2 / W, (y1 + y2) / 2 / H,
                                     (x2 - x1) / W, (y2 - y1) / H))
                    report["boxes_added_hiconf"] += 1
        plan_row = (imf, new_rows)
        # ---------- 切分(复刻 build_real_data.py 纪律)----------
        plan["_tmp"] = plan.get("_tmp", []) + [plan_row]

    grouped = {"clip": [], "frame": [], "v1": []}
    for imf, rows in plan["_tmp"]:
        grouped[series_of(imf.name)].append((imf, rows))
    val_set = set()
    clips_l = [p for p, _ in grouped["clip"] if (SRC / "labels" / (p.stem + ".txt")).exists()]
    val_set.update(random.sample(clips_l, k=round(len(clips_l) * 0.10)))
    frames = sorted(grouped["frame"], key=lambda x: frame_no(x[0].name))
    n_val = max(8, round(len(frames) * 0.10)); mid = len(frames) // 2
    val_set.update(p for p, _ in frames[mid - n_val // 2: mid + n_val // 2])
    v1s = sorted(grouped["v1"], key=lambda x: frame_no(x[0].name))
    n_val = max(6, round(len(v1s) * 0.10))
    val_set.update(p for p, _ in v1s[-n_val:])

    def write_split(split, items):
        for imf, rows in items:
            shutil_copy = imf.read_bytes()
            (DST / "images" / split / imf.name).write_bytes(shutil_copy)
            lines = [f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}" for c, x, y, w, h in rows]
            (DST / "labels" / split / (imf.stem + ".txt")).write_text(
                "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
            if not rows and split == "train":
                report["bg_train"] += 1

    tr = [(p, r) for p, r in plan["_tmp"] if p not in val_set]
    va = [(p, r) for p, r in plan["_tmp"] if p in val_set]
    va = [(p, r) for p, r in va if r]  # val 只放有标注图(与 v2 纪律一致)
    moved = [(p, r) for p, r in va if p in val_set and not r]
    tr += moved
    write_split("train", tr)
    write_split("val", va)
    print(f"哈工大域重标并入:train {len(tr)} + val {len(va)}(val 无标注帧 {len(moved)} 移入 train)")
    print("  重标明细:", report)

    # ---------- 直采组并入 ----------
    n_dtr = n_dva = 0
    for tag, n_val_g in sorted(VAL_PLAN.items()):
        for img in sorted((DIRECT / "images").glob(f"exp{tag}_*.jpg")):
            seq = int(img.stem.split("_")[-1])
            split = "val" if seq < n_val_g else "train"
            lbl = (DIRECT / "labels" / (img.stem + ".txt")).read_text()
            (DST / "images" / split / img.name).write_bytes(img.read_bytes())
            (DST / "labels" / split / (img.stem + ".txt")).write_text(lbl, encoding="utf-8")
            n_dva += split == "val"; n_dtr += split == "train"
    print(f"直采组并入:train {n_dtr} + val {n_dva}")

    (DST / "data.yaml").write_text(
        "train: images/train\nval: images/val\nnames:\n"
        + "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES)), encoding="utf-8")

    print("\n===== v3 数据集统计 =====")
    dist = {"train": Counter(), "val": Counter()}
    for split in ("train", "val"):
        n_img = len(list((DST / "images" / split).glob("*")))
        for lf in (DST / "labels" / split).glob("*.txt"):
            for line in lf.read_text().splitlines():
                if line.strip():
                    dist[split][int(line.split()[0])] += 1
        print(f"{split}: 图 {n_img}  框 {sum(dist[split].values())}  分布 "
              + " ".join(f"{NAMES[i]}={dist[split][i]}" for i in range(10)))
    (DST / "relabel_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    import shutil  # noqa: F401 (write_split 内 read_bytes/write_bytes 已替代)
    main()
