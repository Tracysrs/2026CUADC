# 10_机载代码 · 自研机载软件

自研 ROS 2 代码全部放本目录（一个功能一个包）。方案依据 SSOT（`../01_设计/总体方案与执行计划.md`）；
**要改方案改 SSOT，不在代码注释里另立方案**。每版代码必须先过 SITL 再上真机（SSOT §7 铁律）。

## 目录

| 项 | 内容 | 里程碑状态 |
|---|---|---|
| `cuadc_mission/` | 任务状态机：全生命周期状态机 + 感知接入 + 目标锁定 + 两段对准 + 八门控投放（SSOT §5.1/§5.2） | **M1 骨架 + M3 投放链就绪，待 SITL 验证**（C++ 核心已编译测试 145 项 PASS） |
| `cuadc_interfaces/` | 自定义消息包：侦察判读 `ReconClassification/ReconMarker`（契约 §5）；SafetyStatus 等 M4 接口后续在此追加 | 就绪 |
| `cuadc_perception/` | 感知包：假感知 + 契约校验器（真感知 M2 实现）+ 判读融合库 `recon_fusion.py` + 假判读节点 + 地面站查看器（真判读 M4） | 假节点就绪，融合库已单测 |
| `scripts/` | `setup_env_ubuntu22.sh`（环境一键装）+ `run_sitl.sh`（SITL 全链路一键拉起） | 就绪 |
| `接口契约.md` | 视觉↔状态机↔侦察接口权威文档（字段复用/哨兵/时间戳/心跳/判读消息） | **v1.1 定稿** |
| `时间同步设计.md` | P0.4：取帧时刻戳 + odom 插值夹逼（代码已就绪，M2 仿真验收） | 设计+代码就绪 |

### 侦察判读联调（不需要真模型/飞控）

```bash
cp -r cuadc_interfaces cuadc_perception ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select cuadc_interfaces cuadc_perception
source install/setup.bash
ros2 launch cuadc_perception fake_recon.launch.py                 # 假判读 + 查看器
ros2 launch cuadc_perception fake_recon.launch.py scenario:=ambiguous   # 混淆拒识场景
# 手工触发一次握手（观察 capture_done 与异步判读）：
ros2 topic pub --once /cuadc/recon/capture_request geometry_msgs/msg/PointStamped "{point: {x: 0}}"
# 融合规则单测（本机 Windows 也能跑，无需 ROS）：
PYTHONPATH=cuadc_perception python -m unittest discover -s cuadc_perception/test -v
```

---

## cuadc_mission（任务状态机）

```text
cuadc_mission/
├── src/mission_node.cpp          # 状态机 + tick(20Hz) + 安全门禁 + 感知消费 + 投放决策调用 + 舵机
├── include/cuadc_mission/drop_logic.hpp  # 投放决策核心 C++17（tools/drop_logic.py 的镜像，zig/g++ 编译验证 145 项 PASS）
├── tools/drop_logic.py           # 投放决策核心（纯逻辑规范）：锁定五步/冻结/重捕获/八门控/单发舵机
├── tools/drop_sim.py             # 闭环仿真：运动学+视觉噪声+舵机延迟，蒙特卡洛验证
├── tools/test_drop_logic.py      # 决策核心单元测试（29 例，本机可跑）
├── tools/test_drop_sim.py        # 闭环仿真测试（10 例：基线/断视觉弃桶/投放顺序/压力）
├── test/test_drop_logic.cpp      # hpp 镜像测试（colcon test 自动跑，或 zig/g++ 手动编译）
├── config/mission_params.yaml    # 全部可调参数（默认值 = SSOT §5.2/§5.4 基线，74 项）
├── launch/mission.launch.py      # 单节点启动（MAVROS 需已在跑）
├── CMakeLists.txt / package.xml
```

### 投放决策核心（M3，已验证的算法参考实现）

参照 HIT 冠军版（冻结目标集/一次重捕获/单发舵机）+ 其区赛教训
（两瓶投一点、冻结坐标盲投）设计，防线全部有测试钉死：

```bash
# 决策核心 + 闭环仿真全套测试（本机 Windows 即可跑，无需 ROS）：
python -m unittest discover -s cuadc_mission/tools -v
# 蒙特卡洛评估（改噪声假设/参数后回归）：
cd cuadc_mission/tools && python -c "from drop_sim import *; \
  rs = run_monte_carlo(30); \
  print('lock', sum(r.locked for r in rs), '/', len(rs), \
        'hit', f'{sum(1 for r in rs for l in r.landings if l.hit)/max(1,sum(len(r.landings) for r in rs)):.0%}')"
```

当前基线（种子固定）：锁定 30/30，两瓶 60/60 投放，命中 98%，
CEP 中位 5.0cm，五条安全不变量（单发/新鲜度/不同筒/弃桶拉黑/断视觉禁投）
零违反。**M3 接入 mission_node.cpp 时按 SSOT 铁律先过 SITL**：C++ 移植
须与 `drop_logic.py` 逐函数对应，移植后先跑同一套仿真口径的回归再上真机。

### 状态流转（SSOT §5.1）

```text
准备：WAIT_FCU → WAIT_NAV_STABLE → LOCK_FRAME(航向2s内变≤2°) → PRESTREAM(预发setpoint 1.5s)
      → WAIT_GUIDED(飞手切GUIDED) → WAIT_ARM
任务：TAKEOFF(4m, 0.9×高切setpoint) → SEARCH(2m蛇形) → ALIGN → RELEASE(×2)  [M3]
侦察：RECON_CLIMB → RECON_SURVEY(蛇形6航点+拍照握手) → RETURN_CLIMB → RETURN_HOME(4m)
收尾：LAND(≤0.30m+速度门限+稳定1.5s) → DISARM(自动上锁) → DONE
旁路：PILOT_OVERRIDE(飞手切走GUIDED,程序退让) / ABORT(致命故障)
```

### M1 骨架范围

**已实现**：
- 准备五连 + `LOCK_FRAME` 独立态（航向 2s 内变化 ≤2° 才锁任务坐标系，超时判罗盘故障）
- `CommandTOL` 起飞（0.9×目标高才切 setpoint）、参数化弓字形搜索航线 + 侦察 6 航点（时间插值 smoothstep 平滑）
- 侦察段拍照请求/确认握手（`/cuadc/recon/capture_request` ↔ `capture_done`，1s 超时兜底）
- 降落确认（≤0.30m + 速度门限 + 稳定 1.5s）→ 独立 `DISARM` 态自动上锁 → DONE
- 安全：`flight_gate_ok()` 每飞行态必检、odom 断流 1s 看门狗、任务 240s 超时、
  飞手切走 GUIDED 立即停发目标点退让（`PILOT_OVERRIDE`）、`fail`/`fail_and_return` 两级失败

**未实现（fail-closed，触发即安全返航）**：
- M2 真感知节点：`cuadc_perception` 内实现 YOLOv8n-seg + 单目解算节点（契约 §1 已定，
  消费端——哨兵校验 + P0.4 插值换算世界系——已在 `bucket_callback` 就绪）
- M4 安全监控：`TODO(M4)` —— safety_monitor 独立进程、丢目标降级、证据落盘联调

**M3 已实现（投放链，2026-09-07）**：
- 感知消费：`/perception/drop_buckets_body` 薄契约（frame_id/版本哨兵整帧校验 +
  置信度门控）→ `interpolate_odom()` + `body_to_local_at()` 换算世界系（P0.4）
- SEARCH 锁定五步：`drop_logic::BucketMap`（EMA/确认/独立性合并/直径对号/
  排名稳定 0.8s/拉黑），航线尽头自动降级"2 筒 + 1 未知"
- ALIGN：粗 0.15m → 精 0.08m → 稳定 0.8s；丢视觉一次重捕获，再丢弃桶拉黑，
  超时 12s 弃桶——**绝不按冻结坐标盲投**（门控内建 target_age ≤0.5s）
- RELEASE：八门控连续 2.3s（稳定 0.8 + 保持 1.5）→ `DropSequencer` 单发舵机
  （DO_SET_SERVO，0.7s 后回仓，回仓指令逐拍重试）→ 拉黑 → 下一瓶
- 安全开关：`enable_release_output=false` 干跑（默认，只打日志不发舵机）；
  `m1_no_vision_mode=true` 全链路旁路演练
- 验证状态：算法核心 C++（drop_logic.hpp）**编译并通过 145 项检查**；
  闭环仿真蒙特卡洛（tools/drop_sim.py）锁定 30/30、两瓶 60/60、命中 98%、
  CEP 中位 5.0cm、五条安全不变量零违反。**mission_node.cpp 本体（ROS 依赖）
  本机无法编译——首次 colcon build 需在 Ubuntu/Jetson 上做，再过 SITL 全链路**

### 与参考代码（公开版）的关键差异

| 点 | 公开版 | 本实现 | 依据 |
|---|---|---|---|
| 坐标系锁定 | 藏在导航就绪态里 | `LOCK_FRAME` 独立态，航向 2s 内变 ≤2° | SSOT §5.1（罗盘漂移为 HIT 失利根因） |
| 搜索后行为 | 演示参数直接进对准 | 正式模式 fail-closed；`m1_no_vision_mode=true` 才飞预设航线演练 | M1 里程碑验收需要无视觉全流程 |
| 参数 | 演示值（3m 起飞等） | SSOT §5.4 基线（4.0/2.0m 高度，4.0/2.0/3.0 m/s 速度） | SSOT §5.4 |
| 侦察握手 | 有 | 保留（1s 超时兜底） | SSOT §5.1 / P4-4 |

---

## 编译与运行（Ubuntu 22.04 + ROS 2 Humble + MAVROS）

**一键路径**（首选）：

```bash
./scripts/setup_env_ubuntu22.sh    # 首次：ROS2/Gazebo/MAVROS/SITL/工作空间全装（幂等可重跑）
./scripts/run_sitl.sh              # 每次联调：gz sim → sim_vehicle → MAVROS → 任务节点一键拉起
./scripts/run_sitl.sh --regen      # 重新生成随机比赛场景后再起
```

手动步骤（排查时用）：
```bash
# 1) 把 cuadc_mission/ 整个目录拷到机载电脑（Jetson）的工作空间
mkdir -p ~/ros2_ws/src
cp -r /path/to/10_机载代码/cuadc_mission ~/ros2_ws/src/

# 2) 编译
cd ~/ros2_ws
colcon build --packages-select cuadc_mission
source install/setup.bash

# 3) 联跑顺序：
#    ① Gazebo + ArduPilot SITL（见 08_参考/sim/cuadc_sim 的说明）
#    ② ros2 launch mavros apm.launch
#    ③ 本节点（SITL 用仿真时钟）：
ros2 launch cuadc_mission mission.launch.py use_sim_time:=true

# 看状态流转：
ros2 topic echo /cuadc/mission_state
```

调参：改 `config/mission_params.yaml` 后重新 `colcon build`（config 随包安装），或临时用
`ros2 run cuadc_mission cuadc_mission_node --ros-args --params-file <yaml>`。

## M1 验收清单（对照 SSOT P1-7 / M1"能飞"）

- [x] SITL：起降 + 预设航线全流程无人工干预，状态轨迹 `WAIT_FCU → … → DONE`（9-7 Jetson 实测，ArduPilot SITL 无 Gazebo 轻量链路）
- [x] 飞行中遥控切出 GUIDED：节点停发目标点、打印 `PILOT_OVERRIDE` 并退出（接管保护）（9-7 实测）
- [x] 断定位流（模拟 odom 断流 >1s）：触发 fail 路径安全落地（9-7 SIGSTOP SITL 实测：`MISSION_FAILURE: Odometry stale`）
- [ ] 真机首测（**拆桨**）：起飞 4m → 航线 → 返航 → 降落 → 自动上锁，全程 ≤40s
- [ ] `LOCK_FRAME` 态故意晃动机头：航向漂移超 2° 时锁不上、超时判失败

## 红线

1. 每一版先过 SITL 再上真机；真机首测拆桨、有安全员、有人工接管手段（SSOT §7）。
2. `auto_arm_on_guided` 默认 false，解锁权在飞手。
3. M2 接入感知后，正式训练/比赛 `m1_no_vision_mode` 必须为 false（该演练模式仅用于无视觉排查）。
4. 本机（Windows）未编译验证；首次 `colcon build` 如报笔误，按报错行修正即可。
