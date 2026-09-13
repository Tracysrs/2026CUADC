#!/usr/bin/env python3
"""P0.2/P0.3 TensorRT 部署基准（在 Jetson 上运行）。

流程（SSOT §8.2 部署段 + §9.1 黑帧预热 + P0.2/P0.3 验收线）：
  1. 加载 engine，计时加载耗时；
  2. 【冷启动】第一帧黑帧推理耗时（对照验收线：首帧 <10s）；
  3. 【黑帧预热】连续 3 帧黑帧（上场流程的标准动作，SSOT §9.1），
     记录预热后首帧耗时；
  4. 【稳态】黑帧连推 200 帧，统计平均延迟与 FPS（验收线 ≥25FPS）。

用法：python3 bench_trt.py <engine 或 .pt 路径> [帧数]
退出码 0 = 双验收线通过。TensorRT engine 与权重文件绑定+设备绑定，
换权重/换设备必须重建 engine（SSOT §8.2）。
"""

import sys
import time

import numpy as np
from ultralytics import YOLO

model_path = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/nvidia/cuadc_models/yolov8n-seg.engine'
N_STEADY = int(sys.argv[2]) if len(sys.argv) > 2 else 200

t0 = time.time()
model = YOLO(model_path)
t_load = time.time() - t0

black = np.zeros((640, 640, 3), dtype=np.uint8)   # 黑帧 = 上场前自检标准输入

t1 = time.time()
model.predict(black, imgsz=640, verbose=False)
t_cold = time.time() - t1                          # 冷启动首帧（未预热）

t2 = time.time()
for _ in range(3):                                 # 黑帧预热 3 次
    model.predict(black, imgsz=640, verbose=False)
t_preheat = time.time() - t2

t3 = time.time()
model.predict(black, imgsz=640, verbose=False)     # 预热后首帧（上场实测口径）
t_first_warm = time.time() - t3

t4 = time.time()
for _ in range(N_STEADY):
    model.predict(black, imgsz=640, verbose=False)
dt_avg = (time.time() - t4) / N_STEADY
fps = 1.0 / dt_avg

print('=' * 60)
print(f'模型           : {model_path}')
print(f'加载耗时       : {t_load:.2f} s')
print(f'冷启动首帧     : {t_cold*1000:.0f} ms   (验收 <10000 ms)')
print(f'黑帧预热(3帧)  : {t_preheat*1000:.0f} ms   [P0.3 预热动作]')
print(f'预热后首帧     : {t_first_warm*1000:.0f} ms   (上场实测口径)')
print(f'稳态平均延迟   : {dt_avg*1000:.2f} ms/帧 (含前后处理, {N_STEADY} 帧)')
print(f'稳态 FPS       : {fps:.1f}    (验收 >=25)')
print('=' * 60)
ok = (fps >= 25) and (t_first_warm < 10.0)
print('P0.2/P0.3 验收:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
