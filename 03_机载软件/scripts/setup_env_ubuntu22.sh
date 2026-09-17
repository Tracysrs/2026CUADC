#!/usr/bin/env bash
# =============================================================================
# CUADC 机载开发环境一键安装（Ubuntu 22.04）
#
# 依据：08_参考资料/外部仓库/hgd_cudac/sim/cuadc_sim/docs/SETUP.md 的手工步骤自动化；幂等，可重复执行。
# 用法：
#   ./setup_env_ubuntu22.sh              # 全量安装（含 ArduPilot SITL 编译，约 40 分钟）
#   ./setup_env_ubuntu22.sh --skip-heavy # 跳过 SITL/仿真插件编译（先只装 ROS/MAVROS/Gazebo）
#
# 需要 sudo；建议在 tmux/screen 里跑。装完需要重新登录一次让 dialout 等组生效。
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_TOP="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # 项目根（含 08_参考资料/外部仓库 与 03_机载软件）
SKIP_HEAVY=0
[[ "${1:-}" == "--skip-heavy" ]] && SKIP_HEAVY=1

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
GZ_PLUGIN_DIR="${GZ_PLUGIN_DIR:-$HOME/ardupilot_gazebo}"
WORKSPACE="${WORKSPACE:-$HOME/cuadc_ws}"

log()  { echo -e "\033[32m[setup]\033[0m $*"; }
warn() { echo -e "\033[33m[warn ]\033[0m $*"; }

step_ros2() {
  if [ -f /opt/ros/humble/setup.bash ]; then log "ROS 2 Humble 已安装"; return; fi
  log "安装 ROS 2 Humble …"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop ros-dev-tools
}

step_tools() {
  if ! command -v colcon >/dev/null 2>&1; then
    log "安装 colcon / rosdep …"
    sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
  fi
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init 2>/dev/null || true
  rosdep update --rosdistro humble >/dev/null 2>&1 || warn "rosdep update 有告警，不影响编译"
}

step_gazebo() {
  if gz sim --versions >/dev/null 2>&1; then
    log "Gazebo 已安装: $(gz sim --versions | head -1)"
    return
  fi
  log "安装 Gazebo Harmonic …"
  sudo apt-get install -y curl lsb-release gnupg
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic
}

step_mavros() {
  if bash -c "source /opt/ros/humble/setup.bash && ros2 pkg prefix mavros" >/dev/null 2>&1; then
    log "MAVROS 已安装"
  else
    log "安装 MAVROS …"
    sudo apt-get install -y ros-humble-mavros ros-humble-mavros-extras
  fi
  if [ ! -f /usr/share/geographiclib/geoids/egm96-5.pgm ]; then
    log "安装 geographiclib 数据集（MAVROS 必需）…"
    /opt/ros/humble/share/mavros/install_geographiclib_datasets.sh \
      || warn "数据集脚本失败，请手动执行: /opt/ros/humble/share/mavros/install_geographiclib_datasets.sh"
  else
    log "geographiclib 数据集已存在"
  fi
}

step_ardupilot() {
  if [ "$SKIP_HEAVY" = "1" ]; then warn "跳过 ArduPilot SITL（--skip-heavy）"; return; fi
  if [ -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]; then log "ArduPilot SITL 已编译"; return; fi
  log "克隆并编译 ArduPilot SITL（20~40 分钟）…"
  [ -d "$ARDUPILOT_DIR" ] || git clone --depth 1 https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  cd "$ARDUPILOT_DIR"
  git submodule update --init --recursive
  Tools/environment_install/install-prereqs-ubuntu.sh -y || warn "prereqs 有告警，继续尝试编译"
  # shellcheck disable=SC1091
  . "$HOME/.profile" 2>/dev/null || true
  ./waf configure --board sitl
  ./waf copter -j"$(nproc)"
}

step_gz_plugin() {
  if [ "$SKIP_HEAVY" = "1" ]; then warn "跳过 ardupilot_gazebo 插件（--skip-heavy）"; return; fi
  if find "$GZ_PLUGIN_DIR/build" -name "libArduPilotPlugin.so" 2>/dev/null | grep -q .; then
    log "ardupilot_gazebo 插件已编译"
    return
  fi
  log "编译 ardupilot_gazebo 插件 …"
  sudo apt-get install -y libgz-sim8-dev rapidjson-dev libopencv-dev libopencv-contrib-dev
  [ -d "$GZ_PLUGIN_DIR" ] || git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$GZ_PLUGIN_DIR"
  cmake -S "$GZ_PLUGIN_DIR" -B "$GZ_PLUGIN_DIR/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build "$GZ_PLUGIN_DIR/build" -j"$(nproc)"
}

step_workspace() {
  log "布置 ROS 2 工作空间 $WORKSPACE …"
  mkdir -p "$WORKSPACE/src"
  for pkg in cuadc_mission cuadc_perception; do
    if [ ! -e "$WORKSPACE/src/$pkg" ] && [ -d "$REPO_TOP/03_机载软件/$pkg" ]; then
      cp -r "$REPO_TOP/03_机载软件/$pkg" "$WORKSPACE/src/$pkg"
      log "  拷入 $pkg"
    fi
  done
  # HIT 仿真包：目录叫 cuadc_sim，包名叫 cuadc_rescue_sim，工作区内统一用包名
  if [ -d "$REPO_TOP/08_参考资料/外部仓库/hgd_cudac/sim/cuadc_sim" ] && [ ! -e "$WORKSPACE/src/cuadc_rescue_sim" ]; then
    cp -r "$REPO_TOP/08_参考资料/外部仓库/hgd_cudac/sim/cuadc_sim" "$WORKSPACE/src/cuadc_rescue_sim"
    log "  拷入 cuadc_rescue_sim（来自 外部仓库）"
  fi
  if [ -f "$WORKSPACE/src/cuadc_rescue_sim/scripts/generate_scene.py" ]; then
    (cd "$WORKSPACE/src/cuadc_rescue_sim" && python3 scripts/generate_scene.py) \
      || warn "随机场景生成失败（不阻断，可后补）"
  fi
  log "colcon build …"
  cd "$WORKSPACE"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths src --ignore-src -y || warn "rosdep 有缺项，按提示补装后重跑"
  colcon build
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
  log "构建完成。验证: ros2 pkg executables cuadc_mission"
}

log "==== CUADC 机载环境一键安装开始（仓库: $REPO_TOP）===="
step_ros2
step_tools
step_gazebo
step_mavros
step_ardupilot
step_gz_plugin
step_workspace

log "==== 全部完成 ===="
if ! id -nG "$USER" | grep -qw dialout; then
  warn "当前用户不在 dialout 组（真机串口需要）：sudo usermod -aG dialout \$USER 后重新登录"
fi
echo "下一步：
  1) source $WORKSPACE/install/setup.bash
  2) 一键仿真:   $SCRIPT_DIR/run_sitl.sh          （四件套全链路）
  3) 契约单测:   ros2 launch cuadc_perception fake_perception.launch.py
                 ros2 run cuadc_perception check_vision_contract"
