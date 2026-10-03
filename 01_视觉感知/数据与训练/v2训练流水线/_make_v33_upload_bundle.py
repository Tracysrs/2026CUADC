# -*- coding: utf-8 -*-
"""打包 AutoDL 上传总包 v33_autodl_upload.zip（数据集 zip + 一键脚本 + 预训练权重）。

沿 _make_v32_upload_bundle.py 范式：
- 打包前五道校验：脚本版本标记 / .sh 零 CR 字节 / 预训练权重体量 / 数据集数量对账 / 条目正斜杠
- 数据集 jpg 已压缩,内层用 STORED 免重复压缩;上传后可对照 SHA256
"""
import hashlib
import sys
import zipfile
from pathlib import Path

sys.stdout.reconfigure(encoding='utf-8')
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DATA_DIR = ROOT / 'hazard_v33_direct'
SH = HERE / 'run_v33_training.sh'
PT = ROOT / 'yolov8n.pt'
OUT = HERE / 'v33_autodl_upload.zip'
DS_ZIP = HERE / 'hazard_v33_direct.zip'
EXPECT_TRAIN, EXPECT_VAL = 544, 77

# ---- 1. 脚本校验：版本标记 + 零 CR（Linux 脚本进 CRLF = bash 当场炸） ----
raw = SH.read_bytes()
assert b'\r' not in raw, 'run_v33_training.sh 含 CR 字节——行尾被改坏,先修再打包'
text = raw.decode('utf-8')
for m in ('EXPECT_TRAIN=544', 'EXPECT_VAL=77', 'val_report_newdomain.txt', 'NEW_STEMS'):
    assert m in text, f'run_v33_training.sh 缺标记 {m!r} —— 拿到旧版脚本了！'
print(f'校验 OK: run_v33_training.sh（{len(raw)} 字节,LF 纯净,含新域单列标记）')

# ---- 2. 预训练权重体量（09-30 曾有 3.4MB 截断损坏件,完好应为 ~6.5MB） ----
sz = PT.stat().st_size
assert 5.5e6 < sz < 8e6, f'yolov8n.pt 体量 {sz} 异常——疑似截断损坏件,先重下'
print(f'校验 OK: yolov8n.pt（{sz/1e6:.2f} MB）')

# ---- 3. 数据集数量对账（与建集审计值一致才打包） ----
n_tr = len(list((DATA_DIR / 'images' / 'train').iterdir()))
n_va = len(list((DATA_DIR / 'images' / 'val').iterdir()))
assert n_tr == EXPECT_TRAIN and n_va == EXPECT_VAL, f'数量不符 train={n_tr} val={n_va}'
for split, n in (('train', n_tr), ('val', n_va)):
    for img in (DATA_DIR / 'images' / split).iterdir():
        assert (DATA_DIR / 'labels' / split / (img.stem + '.txt')).exists(), f'缺标签 {img.name}'
print(f'校验 OK: 数据集 train={n_tr} val={n_va},标签零缺失')

# ---- 4. 打数据集内层 zip（STORED：jpg 已压缩,免重复压缩省时;arcname 必须带
#      顶层目录前缀,否则云端 unzip 散落 CWD、脚本按 ${DATA}/ 找不到——AutoDL 首跑实测坑） ----
DS_ZIP.unlink(missing_ok=True)
with zipfile.ZipFile(DS_ZIP, 'w') as z:
    for p in sorted(DATA_DIR.rglob('*')):
        if p.is_file():
            z.write(p, (DATA_DIR.name / p.relative_to(DATA_DIR)).as_posix(),
                    compress_type=zipfile.ZIP_STORED)
with zipfile.ZipFile(DS_ZIP) as z:
    names = z.namelist()
    assert all('\\' not in n for n in names), '数据集 zip 条目含反斜杠（Windows 打包乱名坑）'
    assert all(n.startswith('hazard_v33_direct/') for n in names), \
        '数据集 zip 缺顶层目录前缀——云端解包会散落 CWD（AutoDL 首跑实测坑）'
    assert z.testzip() is None
print(f'内层: {DS_ZIP.name}（{DS_ZIP.stat().st_size/1e6:.1f} MB,{len(names)} 条目,带顶层前缀,全正斜杠）')

# ---- 5. 打总包 ----
OUT.unlink(missing_ok=True)
with zipfile.ZipFile(OUT, 'w') as z:
    z.write(DS_ZIP, DS_ZIP.name, compress_type=zipfile.ZIP_STORED)
    z.write(PT, PT.name, compress_type=zipfile.ZIP_STORED)
    z.write(SH, SH.name, compress_type=zipfile.ZIP_DEFLATED)
with zipfile.ZipFile(OUT) as z:
    assert z.testzip() is None, '总包 zip 自检失败！'
    print(f'\n{OUT}')
    print(f'  大小 {OUT.stat().st_size/1e6:.1f} MB')
    for i in z.infolist():
        print(f'  {i.filename:<28} {i.file_size/1e6:>9.1f} MB')

h = hashlib.sha256(OUT.read_bytes()).hexdigest()
print(f'\nSHA256（上传后可对照）: {h}')
