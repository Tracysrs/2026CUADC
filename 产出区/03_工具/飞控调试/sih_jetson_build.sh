#!/usr/bin/env bash
# SIH 固件 Jetson 一键编译（CUAV-V6X-v2 + SIM_ENABLED，pin dbe79216）
# 用法（Jetson 上）:
#   ./sih_jetson_build.sh <源码包.tar.gz>   # 首次：解包 + 装依赖 + 编译
#   ./sih_jetson_build.sh                   # 已解包：补依赖 + 重新编译
# 产物: ~/sih_firmware_out/arducopter_SIH_CUAV-V6X-v2.apj (+ .bin + SHA256SUMS.txt)
# 红线: 产物只上台架，禁止外场（见 01_设计/SIH台架仿真计划.md）
set -euo pipefail

SRC_DIR="${SRC_DIR:-$HOME/ardupilot-sih}"
SRC_PIN="dbe792162d06cab66c3475fd5556bf7a120f119e"
OUT_DIR="$HOME/sih_firmware_out"

if [ $# -ge 1 ] && [ -n "$1" ]; then
  echo "== [0/5] 解包源码 $1 -> $SRC_DIR"
  mkdir -p "$SRC_DIR"
  tar -xzf "$1" -C "$SRC_DIR" --strip-components=1
fi
cd "$SRC_DIR"

echo "== [1/5] 核验源码锚点"
[ -f libraries/SITL/SITL.h ] || { echo "FATAL: 源码不完整（缺 libraries/SITL/SITL.h）"; exit 1; }
git config --global --add safe.directory "$SRC_DIR" 2>/dev/null || true
ACTUAL=$(git rev-parse HEAD 2>/dev/null || echo none)
echo "HEAD=$ACTUAL"
echo "期望=$SRC_PIN"
[ "$ACTUAL" = "$SRC_PIN" ] || echo "WARN: HEAD 与锚点不一致——SIH 固件将偏离飞行固件同源原则"

echo "== [2/5] apt 依赖（Ubuntu 22.04 自带 gcc-arm-none-eabi 10.3 可用）"
sudo apt-get update -y || echo "WARN: apt update 失败，继续用现有索引"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  build-essential ccache git python3 python3-dev python3-pip gettext libtool \
  gcc-arm-none-eabi g++-arm-none-eabi

echo "== [3/5] python 依赖（empy 3.3.4 是 ChibiOS 构建硬依赖，版本不能高）"
pip3 install --user -q "empy==3.3.4" pyserial pexpect || {
  echo "pip 直连失败，切清华 pypi 镜像重试"
  pip3 install --user -q -i https://pypi.tuna.tsinghua.edu.cn/simple "empy==3.3.4" pyserial pexpect; }

echo "== [4/5] 编译（四轴 X 机架 + Multicopter 仿真类，extra-hwdef 由官方脚本拼装）"
rm -rf build/CUAV-V6X-v2
./Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
  --board CUAV-V6X-v2 --vehicle copter --frame quad --simclass Multicopter

echo "== [5/5] 收产物"
mkdir -p "$OUT_DIR"
B=build/CUAV-V6X-v2/binaries
ls "$B"
cp -v "$B"/arducopter.apj "$OUT_DIR/arducopter_SIH_CUAV-V6X-v2.apj"
[ -f "$B"/arducopter.bin ] && cp -v "$B"/arducopter.bin "$OUT_DIR/arducopter_SIH_CUAV-V6X-v2.bin"
( cd "$OUT_DIR" && sha256sum arducopter_SIH_CUAV-V6X-v2.* | tee SHA256SUMS.txt )
echo "完成。把 $OUT_DIR 拷回 Windows（01_设计/firmware/ 归档），刷入步骤见 03_工具/飞控调试/SIH台架操作.md"
