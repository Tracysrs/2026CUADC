# 仿真环境（本机 Gazebo）

CUADC"多旋翼侦察与救援"赛项的仿真场地。从 `08_参考/hgd_cudac/sim/cuadc_sim` 原样
拷贝为本目录下的 `cuadc_rescue_sim/`（保留 ROS2 包结构，日后可整包拷入 Jetson 的
`~/cuadc_ws/src` 用 colcon 构建），在**本机 Windows 的 Gazebo Sim** 上运行。

本机 Gazebo：`D:\GAZEBO` 用 pixi（conda-forge）安装的 **gz-sim 10（Jetty）**，
与项目选型定案（新版 Gazebo + ArduPilot SITL + MAVROS，见 01_设计 §7）同一体系。

## 使用

- **启动**：双击 `launch-cuadc-sim.bat`（或命令行执行）。自动设置模型路径并加载
  当前布设好的场地；关掉窗口即停止。
- **重新随机布设场地**：

  ```bash
  cd 03_工具/仿真环境/cuadc_rescue_sim
  python scripts/generate_scene.py
  ```

  按 `config/scene.yaml`（场地尺寸 + seed）随机撒 3 投放桶 / 5 侦察桶 / 3 张危化品
  标识，重写 `worlds/cuadc_rescue_single.sdf`，真值落在 `config/generated_scene.yaml`
  （验收对比用）。改 seed 即换一轮布局。
- **D435i 相机话题**（仿真内下视 RGB-D）：`/d435i/image`、`/d435i/depth_image`、
  `/d435i/points`、`/d435i/camera_info`，感知算法可直接订阅。

## 当前状态与限制（2026-09-10）

- 场地全要素载入验证通过：赛场 + 起飞坪 + 3 投放桶 + 5 侦察桶 + 3 危化品贴图 +
  iris_d435i（GUI 实测渲染正常，相机话题出流）。
- **本机没有 ArduPilotPlugin**（ardupilot_gazebo 插件库）：iris 载入时插件报错但
  不阻断，无人机只是不受飞控驱动的自由体——看场地/调相机没问题，不能飞。要跑
  SITL 全流程需在 Jetson（或本机编译 ardupilot_gazebo）+ ArduPilot SITL，见下。
- **gz CLI 消息收发在本机不可用**（conda-forge 包已知问题：`DynamicFactory:
  Unable to parse descriptor set`，`gz topic -e` / `gz service --req` 全部超时），
  但 `gz topic -l` / `gz service -l` 列举可用，仿真本身不受影响。验证以 GUI /
  话题列表为准。
- ⚠️ **`GZ_SIM_RESOURCE_PATH` 不能留空尾项**（尾部 `;`）：本 gz 构建会把整个路径
  解析搞坏、所有模型静默加载失败（世界只剩 clock/stats，GUI 空实体树）。快速判别：
  `gz topic -l | grep d435i`，有话题=加载成功；若还出现 `/world/shapes` 说明
  world 参数没传对、回退了默认示例世界。

## Jetson SITL（✅ 已打通，2026-09-10 仿真飞行 FLIGHT_OK）

Jetson 侧脚本在 `jetson/` 子目录（Jetson 上同步放于 `~/sim_scripts/`）：

- `start_sitl.sh`：sim_vehicle（gazebo-iris JSON 帧，MAVProxy 出口 UDP 14550）
- `start_gz.sh [world]`：无头 gz 服务器，默认纯物理飞行 world（无渲染，不占 GPU）
- `start_gzgui.sh [world]`：一键挂 Gazebo GUI 到活 server（DISPLAY=:0，带窗口自检）
- `fly.py`：参数自检（FRAME_CLASS=1/TYPE=0）→ GUIDED → 解锁 → 起飞 8 m → 降落
  （裸 SITL 直连：`python3 fly.py tcp:127.0.0.1:5760`，脚本自带数据流请求）
- `fcu_ready.py`：FCU 就绪门禁——直连 5762 探心跳连续稳定（默认 30 s 窗/600 s
  超时），跳过 reset 后分钟级启动不稳定窗；reset 后先跑它（任务脚本已内置同款
  门禁）再开飞
- `fc_sitl_mission.sh [秒数]`：**全任务 M1 闭环**（2026-09-11 全程跑通 DONE，
  2026-09-12 zd550 机体验证 DONE）——起跑前自检 5760 占用 + FCU 心跳稳定窗 →
  MAVROS + set_message_interval + 状态机（auto_arm_on_guided + m1_no_vision）
  + 外部切 GUIDED 作 WAIT_GUIDED 放行扳机；轨迹 TAKEOFF→SEARCH 预设航线→
  RECON_SURVEY 6 航点拍照→RETURN_HOME→LAND→DISARM→DONE。无视觉模式下
  投放 0/2、拍照确认 0 属预期（ALIGN/RELEASE 旁路，拍照 1s 超时兜底）
- `cuadc_rescue_flight.sdf`：布设场景 + 内联碰撞地面（场地模型本体无碰撞体，
  裸跑会穿地）+ 已移除 Sensors 渲染系统

实测：GUIDED 解锁 → 起飞 → 30 s 爬升 55 m → LAND → 落地自动上锁。带 D435i 相机
话题的渲染版 world 用 `start_gz.sh ~/cuadc_ws/src/cuadc_rescue_sim/worlds/cuadc_rescue_single.sdf`
（需 `DISPLAY=:0`）。插件与安装细节、全部排障记录见 2026-09-10 日志。

## 迁移到 Jetson 跑 SITL（✅ 已完成，记录留档）

1. 整包拷 `cuadc_rescue_sim/` 到 `~/cuadc_ws/src/`，`colcon build --packages-select
   cuadc_rescue_sim`。
2. 装配 **ardupilot_gazebo** 插件（提供 ArduPilotPlugin 系统库），iris 才能被
   ArduPilot SITL 驱动。
3. ⚠️ 改 `launch/cuadc_sim.launch.py` 里硬编码的上游路径
   `/home/accelerate/ardupilot_gazebo/...`（原作者用户名）为实际路径；上游的
   `scripts/start_cuadc_sim.sh` 同样硬编码了 `/home/accelerate/cuadc_ws`。
4. ArduPilot SITL：`sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON ...`
   （gz 版插件走 JSON 接口；Jetson 已装 ArduPilot SITL，见 2026-09-07 日志 §四）。

## 版权与来源

模型与贴图来自 `08_参考/hgd_cudac`（上游基于 ardupilot_gazebo 的 Iris 派生，
LGPL——对外发布须保留 `cuadc_rescue_sim/NOTICE.md` 与来源声明；危化品贴图出自
比赛附件 11，仅限训练用途）。按仓库白名单原则，`models/`（22MB mesh/贴图）已加入
`.gitignore` 只留本地，脚本与文档入库。
