# -*- coding: utf-8 -*-
"""台架帧全量预标注批跑：images/*.jpg -> 同名 JSON（X-AnyLabeling）+ 预览叠加 + 报表 CSV。

用法：python run_preannotate_bench_batch.py [图像目录]
  默认图像目录 = <仓>/01_视觉感知/数据与训练/images（脚本位于 barrel_seg_tools/）。
产出：<图像目录>/<同名>.json、<数据与训练>/_preview_annot/annot_*.jpg、<数据与训练>/_batch_report.csv。
口径与流程：preannotate_barrel_bench.py 头注 +《标注要求-白筒seg.md》。
"""
import os, sys, json, csv, time
import cv2, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import preannotate_barrel_bench as A          # noqa: E402
from ultralytics import YOLO, SAM             # noqa: E402

DATA_ROOT = os.path.normpath(os.path.join(HERE, '..'))
IMG_DIR = os.path.join(DATA_ROOT, 'images')
PREV = os.path.join(DATA_ROOT, '_preview_annot')


def main():
    img_dir = sys.argv[1] if len(sys.argv) > 1 else IMG_DIR
    os.makedirs(PREV, exist_ok=True)
    model = YOLO(A.MODEL)
    sam = SAM(os.path.join(DATA_ROOT, 'mobile_sam.pt'))
    files = sorted(f for f in os.listdir(img_dir) if f.endswith('.jpg'))
    print('total', len(files), flush=True)

    rows = []
    t00 = time.time()
    for i, fn in enumerate(files):
        path = os.path.join(img_dir, fn)
        try:
            shapes, vis, dbg = A.annotate_image(model, path, sam=sam)
        except Exception as e:
            rows.append([fn, 'ERROR', str(e)[:80]])
            print(i, fn, 'ERROR', e, flush=True)
            continue
        if shapes is None:
            rows.append([fn, 'READ-FAIL', ''])
            continue
        img = cv2.imread(path)
        j = A.to_xanylabeling(shapes, path, img.shape[1], img.shape[0])
        with open(os.path.join(img_dir, fn[:-4] + '.json'), 'w', encoding='utf-8') as f:
            json.dump(j, f, ensure_ascii=False, indent=2)
        if vis is not None:
            A.imwrite_u(os.path.join(PREV, 'annot_' + fn), vis)
        desc = '|'.join('%s:%.2f%s' % (s['how'], s['conf'], 'D' if s['difficult'] else '') for s in shapes)
        rows.append([fn, len(shapes), desc])
        if i % 20 == 0:
            print('%d/%d %.0fs' % (i, len(files), time.time() - t00), flush=True)

    rp = os.path.join(DATA_ROOT, '_batch_report.csv')
    with open(rp, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['file', 'n_shapes', 'shapes'])
        w.writerows(rows)
    print('DONE', time.time() - t00, 's ->', rp, flush=True)


if __name__ == '__main__':
    main()
