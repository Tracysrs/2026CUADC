#!/usr/bin/env bash
# =============================================================================
# TensorRT 部署一键脚本（在 Jetson 上运行）· 多模型版
#
# 用法：bash deploy_trt_jetson.sh <weights.pt> [slot] [基准帧数]
#   slot ∈ recon(默认→best.engine) | bucket(→bucket.engine) | hcircle(→h_circle.engine)
# 流程：.pt → TRT engine(FP16, imgsz 640) → 移入 ~/cuadc_models/<slot>.engine
#       → SHA-256 登记入 SHA256SUMS.txt → bench_trt.py 实测
#
# 模型三件（设计文档 §3 架构）：
#   recon    危化判读 10 类 det（hazard_recon_node，已生产）
#   bucket   白筒分割单类 seg（bucket_perception_node，M2）
#   hcircle  H 圆检测单类 det（h_circle_node，M4）
# 部署后把打印的 SHA-256 回填对应 config/*_params.yaml 的 expected_sha256，
# 然后重启对应服务（systemctl restart cuadc-perception 等）。
#
# 前置（JetPack 6.x 自带）：TensorRT 10.x + torch(NVIDIA 轮子) + cv2；
# 需 pip 装：onnx onnxslim。
#
# ⚠️ 铁律（9-7/9-12 实测教训）：
#   1. numpy 必须 <2 —— 系统 cv2/TensorRT 是 numpy1 ABI，pip 升级即炸；
#   2. engine 与【权重+设备】双重绑定——换权重、换设备都要重建（约 10 分钟，
#      比赛日绝不可现场构建）；
#   3. 上场前必须黑帧预热（首帧冷启动有秒级 CUDA/TRT 初始化）。
# =============================================================================
set -e

W="${1:?用法: bash deploy_trt_jetson.sh <weights.pt> [slot] [基准帧数]}"
SLOT="${2:-recon}"
N="${3:-200}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODELS_DIR="$HOME/cuadc_models"

case "$SLOT" in
  recon)   ENGINE_NAME="best.engine" ;;
  bucket)  ENGINE_NAME="bucket.engine" ;;
  hcircle) ENGINE_NAME="h_circle.engine" ;;
  *) echo "未知 slot: $SLOT（recon|bucket|hcircle）"; exit 1 ;;
esac

# 依赖保障（幂等；numpy<2 是硬约束，见上）；走清华镜像——Jetson 直连 pypi 会卡死（9-12 实测）
pip install -q -i https://pypi.tuna.tsinghua.edu.cn/simple 'numpy<2' onnx onnxslim

cd "$(dirname "$W")"
echo "=== [$SLOT] 导出 TRT engine (FP16, imgsz 640)——约 10 分钟 ==="
yolo export model="$W" format=engine half=True imgsz=640 device=0 exist_ok=True

RAW_ENGINE="${W%.pt}.engine"
mkdir -p "$MODELS_DIR"
ENGINE="$MODELS_DIR/$ENGINE_NAME"
mv -f "$RAW_ENGINE" "$ENGINE"
ONNX="${W%.pt}.onnx"; [ -f "$ONNX" ] && mv -f "$ONNX" "$MODELS_DIR/"

SHA=$(sha256sum "$ENGINE" | cut -d' ' -f1)
# SHA 台账：先清掉本槽旧行再追加（grep 反向匹配防 self-match）
grep -v "  $ENGINE_NAME$" "$MODELS_DIR/SHA256SUMS.txt" 2>/dev/null > "$MODELS_DIR/SHA256SUMS.txt.tmp" || true
mv "$MODELS_DIR/SHA256SUMS.txt.tmp" "$MODELS_DIR/SHA256SUMS.txt"
echo "$SHA  $ENGINE_NAME" >> "$MODELS_DIR/SHA256SUMS.txt"
# 权重归档（换 weights 前留底）
cp -f "$W" "$MODELS_DIR/archive/${SLOT}_$(date +%Y%m%d)_$(basename "$W")" 2>/dev/null || true

echo "=== [$SLOT] engine 就绪: $ENGINE ==="
echo "    SHA-256: $SHA"
echo "    ⚠️ 回填 expected_sha256："
case "$SLOT" in
  recon)   echo "       cuadc_perception/config/hazard_recon_params.yaml" ;;
  bucket)  echo "       cuadc_perception/config/bucket_perception_params.yaml" ;;
  hcircle) echo "       cuadc_perception/config/h_circle_params.yaml" ;;
esac

echo "=== 基准实测 ==="
python3 "$SCRIPT_DIR/bench_trt.py" "$ENGINE" "$N"
