# -*- coding: utf-8 -*-
"""CUADC 2026 危化标识检测 · v3.3 建集脚本(hazard_v32_direct + data1001 合并)。

v3.2 数据(hazard_v32_direct,516 train + 67 val)原样作底,本脚本不改其任何文件;
data1001(2026-10-01 机载斜视/运动模糊/暗光批,38 张,标注见该目录 README)按防泄漏
纪律并入。域价值 = v3.2 三域(哈工大棚拍/09-29 直采/09-30 夜暗全摆)之外的第四域:
无人机低空斜视 + 运动模糊 + 傍晚暗光 + 地面杂物负样本(鞋/椅/工具车)。
本批固定五牌摆位,仅覆盖 c2刺激性/c3放射性/c4腐蚀品/c7有毒品/c8自燃物品 五类。

data1001 切分纪律(两连拍块,块内帧间隔 2s,块间 54s,按块尾整段切 val):
  val = A块尾 191712~191720(5 帧,远距模糊视点) + B块尾 191845~191853(5 帧,近距清晰视点)
  train = 其余 28 帧(两块都有 train 帧)
  注:块内边界与相邻 train 帧仅隔 2s(连拍块内切割,同 v3 直采组先例),
      比 data0930「块边界 ≥10s」宽;若要最严可改整块 B 进 val(建集脚本改 VAL_STEMS 即可)。

用法:python build_v33_dataset.py   (产出 hazard_v33_direct/,不动 v3.2 与 data1001)
"""
import shutil
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
V32 = HERE / "hazard_v32_direct"
DATA = HERE / "data1001"
DST = HERE / "hazard_v33_direct"
NAMES = ["爆炸品", "不燃气体", "刺激性", "放射性物品", "腐蚀品",
         "生物危害", "遇湿易燃物品", "有毒品", "自燃物品", "易燃"]
# data1001 val 时段块(两连拍块尾各 5 帧;A尾=远距模糊,B尾=近距清晰)
VAL_STEMS = {
    "191712_015", "191714_016", "191716_017", "191718_018", "191720_019",
    "191845_015", "191847_016", "191849_017", "191851_018", "191853_019",
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

    # 1) hazard_v32_direct 原样搬入(哈工大重映射 + 09-29 直采 + data0930 夜暗)
    for split in ("train", "val"):
        for img in sorted((V32 / "images" / split).iterdir()):
            assert img.name not in seen, f"重名: {img.name}"
            seen.add(img.name)
            lbl = V32 / "labels" / split / (img.stem + ".txt")
            assert lbl.exists(), f"v3.2 标签缺失: {lbl}"
            shutil.copy2(img, DST / "images" / split / img.name)
            shutil.copy2(lbl, DST / "labels" / split / lbl.name)
            stats[split] += count_boxes(lbl)
            n_img[split] += 1

    # 2) data1001 并入(38 张;val 10 帧按声明块,其余 28 进 train)
    frames = sorted(DATA.glob("[0-9][0-9][0-9][0-9][0-9][0-9]_*.jpg"))
    assert len(frames) == 38, f"data1001 应 38 张(39 剔 1 糊帧),实 {len(frames)}"
    for img in frames:
        assert img.stem not in seen, f"重名: {img.stem}"
        seen.add(img.stem)
        split = "val" if img.stem in VAL_STEMS else "train"
        lbl = DATA / "labels" / (img.stem + ".txt")
        assert lbl.exists(), f"data1001 标签缺失: {lbl}"
        shutil.copy2(img, DST / "images" / split / img.name)
        shutil.copy2(lbl, DST / "labels" / split / lbl.name)
        stats[split] += count_boxes(lbl)
        n_img[split] += 1
    assert n_img["val"] == 67 + 10, f"val 应 77 图,实 {n_img['val']}"
    assert n_img["train"] == 516 + 28, f"train 应 544 图,实 {n_img['train']}"

    # 3) data.yaml(与 v3 同表,无 path 行——云端由 run_v33_training.sh sed 重写)
    (DST / "data.yaml").write_text(
        "train: images/train\nval: images/val\nnames:\n"
        + "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES)),
        encoding="utf-8")

    print(f"hazard_v33_direct 建集完成: train {n_img['train']} 图 / val {n_img['val']} 图")
    for split in ("train", "val"):
        total = sum(stats[split].values())
        dist = {f"{i}{NAMES[i]}": stats[split][i] for i in range(10)}
        print(f"  {split}: {total} 框 {dist}")


if __name__ == "__main__":
    main()
