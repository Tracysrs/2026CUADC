#!/usr/bin/env bash
# =============================================================================
# 台架 A/B 影子 engine 构建（在 Jetson 上运行）· 不碰生产
#
# 用法：bash bench_ab_setup.sh <weights.pt> [imgsz]
#   例：bash bench_ab_setup.sh ~/bench_ab/best_v32.pt
# 流程：<name>.pt → TRT engine(FP16, 设备上导出, 与 deploy_trt_jetson.sh 同路径)
#       → ~/v3_shadow/<name>.engine → 软链 ~/v3_shadow/best_v3.engine（live_ab 读取）
#       → SHA-256 记 ~/v3_shadow/SHA256SUMS.txt
#
# ⚠️ 铁律：
#   1. 全程只写 ~/v3_shadow/，**绝不触碰 ~/cuadc_models/（生产 v2）**；
#   2. engine 与【权重+设备】双重绑定——换权重/换设备都要重建（约 10 分钟/个）；
#   3. numpy 必须 <2（系统 cv2/TensorRT 是 numpy1 ABI）；清华镜像（直连 pypi 卡死）。
# 切换 A/B 对象：ln -sfn ~/v3_shadow/best_v31.engine ~/v3_shadow/best_v3.engine
# =============================================================================
set -e

W="${1:?用法: bash bench_ab_setup.sh <weights.pt> [imgsz=640]}"
IMGSZ="${2:-640}"
SHADOW="$HOME/v3_shadow"
NAME="$(basename "$W" .pt)"

pip install -q -i https://pypi.tuna.tsinghua.edu.cn/simple 'numpy<2' onnx onnxslim

mkdir -p "$SHADOW"
cd "$(dirname "$W")"
echo "=== [$NAME] 导出影子 TRT engine (FP16, imgsz $IMGSZ)——约 10 分钟 ==="
yolo export model="$W" format=engine half=True imgsz=$IMGSZ device=0 exist_ok=True

ENGINE="$SHADOW/${NAME}.engine"
mv -f "${W%.pt}.engine" "$ENGINE"
ONNX="${W%.pt}.onnx"; [ -f "$ONNX" ] && mv -f "$ONNX" "$SHADOW/"

SHA=$(sha256sum "$ENGINE" | cut -d' ' -f1)
grep -v "  ${NAME}.engine$" "$SHADOW/SHA256SUMS.txt" 2>/dev/null > "$SHADOW/SHA256SUMS.txt.tmp" || true
mv "$SHADOW/SHA256SUMS.txt.tmp" "$SHADOW/SHA256SUMS.txt"
echo "$SHA  ${NAME}.engine" >> "$SHADOW/SHA256SUMS.txt"

# live_ab.py 固定读 ~/v3_shadow/best_v3.engine——软链切到刚构建的这份
ln -sfn "$ENGINE" "$SHADOW/best_v3.engine"

echo "=== [$NAME] 影子 engine 就绪 ==="
echo "    engine : $ENGINE"
echo "    SHA-256: $SHA"
echo "    软链   : $SHADOW/best_v3.engine → $ENGINE"
echo "    生产未动：~/cuadc_models/ 本脚本零写入（A/B 判读全程走影子）"
echo ""
echo "下一步："
echo "  nohup python3 ~/bench_ab/live_ab.py > /tmp/live_ab.log 2>&1 &   浏览器 http://<jetson-ip>:8080"
echo "  基准：python3 <scripts目录>/bench_trt.py $ENGINE 200"
echo "  换 A/B 对象：ln -sfn ~/v3_shadow/best_v31.engine ~/v3_shadow/best_v3.engine"
