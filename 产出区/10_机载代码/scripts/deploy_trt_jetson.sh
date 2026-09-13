#!/usr/bin/env bash
# =============================================================================
# P0.2 TensorRT 部署一键脚本（在 Jetson 上运行）
#
# 用法：bash deploy_trt_jetson.sh <weights.pt> [基准帧数，默认 200]
# 流程：.pt → ONNX → TRT engine(FP16, imgsz 640) → bench_trt.py 实测
# 产物：与权重同目录的 .onnx / .engine
#
# 前置（JetPack 6.x 自带）：TensorRT 10.x + torch(NVIDIA 轮子) + cv2；
# 需 pip 装：onnx onnxslim。
#
# ⚠️ 铁律（9-7 实测教训）：
#   1. numpy 必须 <2 —— 系统 cv2/TensorRT 是 numpy1 ABI，pip 升级即炸；
#   2. engine 与【权重+设备】双重绑定——换权重、换设备都要重建（约 10 分钟，
#      比赛日绝不可现场构建）；
#   3. 上场前必须黑帧预热（首帧冷启动有秒级 CUDA/TRT 初始化）。
# =============================================================================
set -e

W="${1:?用法: bash deploy_trt_jetson.sh <weights.pt> [基准帧数]}"
N="${2:-200}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 依赖保障（幂等；numpy<2 是硬约束，见上）；走清华镜像——Jetson 直连 pypi 会卡死（9-12 实测）
pip install -q -i https://pypi.tuna.tsinghua.edu.cn/simple 'numpy<2' onnx onnxslim

cd "$(dirname "$W")"
echo "=== 导出 TRT engine (FP16, imgsz 640)——约 10 分钟 ==="
yolo export model="$W" format=engine half=True imgsz=640 device=0 exist_ok=True

ENGINE="${W%.pt}.engine"
echo "=== 基准实测 ==="
python3 "$SCRIPT_DIR/bench_trt.py" "$ENGINE" "$N"
