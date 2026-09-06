# -*- coding: utf-8 -*-
"""
CUADC 2026 危化标识检测 · 实拍数据集构建脚本（v2 混合微调用）
把 train/ 原始数据整理成 ultralytics 可直接训练的 real_data/ 结构。

关键处理：
1. 剔除 31 张哈希命名 PNG（网页/文档截图，与任务无关，均无标注）
2. train/val 按视频片段切分，避免连续帧泄题导致 mAP 虚高：
   - NNNN_frame_* 系列：101 个片段各 1 帧，随机抽 10% 作 val
   - frame_* 系列（单一长视频）：取中间一段连续帧作 val
   - 1_* 系列（单一长视频）：取末尾连续帧作 val
   - val 只放有标注的图；无标注实拍帧全部进 train 当背景图（补空标签文件）
3. data.yaml 不写 path 键：ultralytics 会以 yaml 所在目录为根，PC/AutoDL 通用
"""
import random
import re
import shutil
from collections import Counter
from pathlib import Path

random.seed(42)

_HERE = Path(__file__).resolve().parent
SRC = _HERE / "train"
DST = _HERE / "real_data"

CLASS_NAMES = {
    0: "爆炸品", 1: "不燃气体", 2: "腐蚀品", 3: "刺激性", 4: "感染性物品",
    5: "氧化剂", 6: "遇湿易燃物品", 7: "有毒品", 8: "自燃物品", 9: "易燃液体",
    10: "barrel",
}

RE_CLIP = re.compile(r"^\d{4}_frame_\d+\.(jpg|png)$")
RE_V1 = re.compile(r"^1_\d+\.(jpg|png)$")

def series_of(name: str) -> str:
    if RE_CLIP.match(name):
        return "clip"
    if name.startswith("frame_"):
        return "frame"
    if RE_V1.match(name):
        return "v1"
    return "other"

def frame_no(name: str) -> int:
    m = re.search(r"(\d+)\.(jpg|png)$", name)
    return int(m.group(1)) if m else 0

def main():
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (DST / sub).mkdir(parents=True, exist_ok=True)

    images = sorted((SRC / "images").iterdir())
    grouped = {"clip": [], "frame": [], "v1": [], "other": []}
    for img in images:
        grouped[series_of(img.name)].append(img)

    plan = {"train": [], "val": []}  # (image_path, has_label)

    # clip 系列：每个片段只有 1 帧，随机切不泄题；val 只取有标注的
    clips = grouped["clip"]
    clips_labeled = [p for p in clips if (SRC / "labels" / (p.stem + ".txt")).exists()]
    val_clips = set(random.sample(clips_labeled, k=round(len(clips_labeled) * 0.10)))
    for p in clips:
        plan["val" if p in val_clips else "train"].append(p)

    # frame 系列：中间取 ~10% 连续帧作 val
    frames = sorted(grouped["frame"], key=lambda p: frame_no(p.name))
    n_val = max(8, round(len(frames) * 0.10))
    mid = len(frames) // 2
    val_frames = set(frames[mid - n_val // 2 : mid + n_val // 2])
    for p in frames:
        plan["val" if p in val_frames else "train"].append(p)

    # 1_ 系列：末尾取 ~10% 连续帧作 val
    v1s = sorted(grouped["v1"], key=lambda p: frame_no(p.name))
    n_val = max(6, round(len(v1s) * 0.10))
    val_v1 = set(v1s[-n_val:])
    for p in v1s:
        plan["val" if p in val_v1 else "train"].append(p)

    # other（哈希 PNG 等）：剔除，不进数据集
    print(f"剔除无关图片 {len(grouped['other'])} 张：")
    for p in grouped["other"]:
        print(f"  - {p.name}")

    n_bg = 0
    for split, items in plan.items():
        for img in items:
            label_src = SRC / "labels" / (img.stem + ".txt")
            shutil.copy2(img, DST / "images" / split / img.name)
            if label_src.exists():
                shutil.copy2(label_src, DST / "labels" / split / (img.stem + ".txt"))
            else:
                (DST / "labels" / split / (img.stem + ".txt")).write_text("")
                n_bg += 1

    # 写 data.yaml：不写 path，ultralytics 以本文件所在目录为根
    lines = ["train: images/train", "val: images/val", "names:"]
    lines += [f"  {k}: {v}" for k, v in CLASS_NAMES.items()]
    (DST / "data.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # 分布统计
    print(f"\n背景图（无标注，进 train）{n_bg} 张\n")
    for split in ("train", "val"):
        imgs = list((DST / "images" / split).iterdir())
        cnt = Counter()
        for lf in (DST / "labels" / split).glob("*.txt"):
            for line in lf.read_text().splitlines():
                if line.strip():
                    cnt[int(line.split()[0])] += 1
        print(f"[{split}] 图片 {len(imgs)} 张，实例 {sum(cnt.values())} 个")
        for cid in range(11):
            print(f"  {cid} {CLASS_NAMES[cid]:　<6} {cnt.get(cid, 0)}")

if __name__ == "__main__":
    main()
