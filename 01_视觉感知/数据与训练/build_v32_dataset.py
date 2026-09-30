# -*- coding: utf-8 -*-
"""CUADC 2026 危化标识检测 · v3.2 建集脚本(v3_direct + data0930 合并)。

哈工大数据的旧表重映射已在 v3 建集完成(build_v3_dataset.py,hazard_v3_direct 即
「重映射后哈工大 376 train + 38 val」+「aircraft_direct_20260929 直采 100 + 20」),
本脚本不改哈工大数据,只做合并:data0930(2026-09-30 夜暗十类全摆场 49 张,标注见
该目录 README)按防泄漏纪律并入。

data0930 切分纪律(帧间隔 7~39s、无连拍块,按互不相邻时段整段切,块间边界 ≥10s):
  val = 233803~233833(4) + 234336~234359(3) + 234513~234521(2) = 9 帧
  train = 其余 40 帧
比 v3 直采组「连拍块内 0.5s 相邻」的切分更严;每组十类全有,val 每类 9 框。

用法:python build_v32_dataset.py   (产出 hazard_v32_direct/,不动 v3_direct 与 data0930)
"""
import shutil
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
V3 = HERE / "hazard_v3_direct"
DATA = HERE / "data0930"
DST = HERE / "hazard_v32_direct"
NAMES = ["爆炸品", "不燃气体", "刺激性", "放射性物品", "腐蚀品",
         "生物危害", "遇湿易燃物品", "有毒品", "自燃物品", "易燃"]
# data0930 val 时段块(与相邻 train 帧间隔:前 10s/后 33s;39s/11s;22s)
VAL_STEMS = {
    "233803_000", "233813_000", "233824_000", "233833_000",
    "234336_000", "234344_000", "234359_000",
    "234513_000", "234521_000",
}


def count_boxes(lbl: Path) -> Counter:
    c = Counter()
    for ln in lbl.read_text(encoding="utf-8").splitlines():
        if ln.strip():
            c[int(ln.split()[0])] += 1
    return c


def main():
    assert DST.exists() is False, f"{DST} 已存在,防误覆盖先确认"
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (DST / sub).mkdir(parents=True, exist_ok=True)

    seen = set()
    stats = {"train": Counter(), "val": Counter()}
    n_img = {"train": 0, "val": 0}

    # 1) v3_direct 原样搬入(哈工大重映射 + 09-29 直采,类别表一致)
    for split in ("train", "val"):
        for img in sorted((V3 / "images" / split).iterdir()):
            assert img.name not in seen, f"重名: {img.name}"
            seen.add(img.name)
            lbl = V3 / "labels" / split / (img.stem + ".txt")
            assert lbl.exists(), f"v3 标签缺失: {lbl}"
            shutil.copy2(img, DST / "images" / split / img.name)
            shutil.copy2(lbl, DST / "labels" / split / lbl.name)
            stats[split] += count_boxes(lbl)
            n_img[split] += 1

    # 2) data0930 并入(49 张;val 9 帧按声明块,其余进 train)
    frames = sorted(DATA.glob("[0-9][0-9][0-9][0-9][0-9][0-9]_*.jpg"))
    assert len(frames) == 49, f"data0930 应 49 张,实 {len(frames)}"
    for img in frames:
        assert img.stem not in seen, f"重名: {img.stem}"
        seen.add(img.stem)
        split = "val" if img.stem in VAL_STEMS else "train"
        lbl = DATA / "labels" / (img.stem + ".txt")
        assert lbl.exists(), f"data0930 标签缺失: {lbl}"
        shutil.copy2(img, DST / "images" / split / img.name)
        shutil.copy2(lbl, DST / "labels" / split / lbl.name)
        stats[split] += count_boxes(lbl)
        n_img[split] += 1
    assert n_img["val"] == 38 + 20 + 9, f"val 应 67 图,实 {n_img['val']}"
    assert n_img["train"] == 376 + 100 + 40, f"train 应 516 图,实 {n_img['train']}"

    # 3) data.yaml(与 v3 同表)
    (DST / "data.yaml").write_text(
        "train: images/train\nval: images/val\nnames:\n"
        + "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES)),
        encoding="utf-8")

    print(f"hazard_v32_direct 建集完成: train {n_img['train']} 图 / val {n_img['val']} 图")
    for split in ("train", "val"):
        total = sum(stats[split].values())
        dist = {f"{i}{NAMES[i]}": stats[split][i] for i in range(10)}
        print(f"  {split}: {total} 框 {dist}")


if __name__ == "__main__":
    main()
