# SIH 台架操作（真飞控跑全任务仿真 + 真舵机/电机输出）

> 背景与版本锚点见 `04_仿真/SIH台架/SIH台架仿真计划.md`。SIH = 仿真物理模型跑在真 V6X 内，
> 行为对齐 SITL；`SIM_OH_MASK` 可把仿真输出打到真 PWM 口（舵机/电机真动）。
> 感知说明：**SIH 不出相机图像**——任务跑通靠 09-12 的"真值感知替身"按契约 v1.3
> 发目标（或 LAB 节点对空发心跳），真相机/模型链的验证不在此范围内。

## 0. 红线（每次上电前默念）

1. **全程拆桨**——`SIM_OH_MASK` 生效后 PWM 口随时带输出，电机位开档（783）后电机随时转；
2. SIH 固件**禁止外场**，只上台架；
3. 用完必恢复：重刷锁定 beta 固件 + 真机参数（§5），跳过 = 参数被污染返工。

## 0.5 固件哪来：Jetson 一键编译（2026-09-15 路线定案）

- Windows 侧已备好源码包（pin `dbe79216`，含 .git）与一键脚本
  `sih_jetson_build.sh`（本目录）；Windows→Jetson 传输用 scp/USB 均可；
- Jetson 上执行：`./sih_jetson_build.sh <源码包.tar.gz>`（首次）/ `./sih_jetson_build.sh`（重编），
  自动完成 apt 依赖（gcc-arm-none-eabi）→ empy 3.3.4 → 官方 sitl-on-hw.py
  （`--board CUAV-V6X-v2 --vehicle copter --frame quad --simclass Multicopter`）；
- 产物 `~/sih_firmware_out/arducopter_SIH_CUAV-V6X-v2.apj` + SHA256 回拷归档到
  `02_飞控与硬件/firmware/` 后再进 §2 刷入。

## 1. 备份（刷机前，一次）

```bash
python ops.py            # 选 params-export —— 全量备份到 02_飞控与硬件/参数备份/
python check_fc.py       # 记录当前固件版本，作为恢复后的对照
```

## 2. 刷入 SIH 固件

1. Mission Planner → 设置 → 安装固件 → 加载自定义固件 →
   选 `02_飞控与硬件/firmware/arducopter_beta_CUAV-V6X-v2_SIH_台架专用.apj`；
2. 重插 USB 重连；
3. 导入 `02_飞控与硬件/V6X_ardupilot_params.param` → 写参数 → 重启 →
   `python verify_params.py --diff 02_飞控与硬件/V6X_ardupilot_params.param` 全项一致；
4. 导入 `04_仿真/SIH台架/V6X_SIH_bench.param` → 写参数 → 重启；
5. 若 SIH 参数没落地：`FORMAT_VERSION=0` → 重启（清参数存储让构建期默认值生效），
   再执行第 3/4 步；
6. **黑活验收**：`check_fc.py` 心跳正常；模式含 GUIDED；回读
   `AHRS_EKF_TYPE=10 / GPS1_TYPE=100 / SIM_OH_MASK=768` 全对才算过。

## 3. 解锁前置

- RC：真遥控器（09-12 已实测通）或 MAVLink RC override；
- GCS 地图上位置在 `SIM_OPOS_*` 设的起点（默认值=任意）；EKF 就绪后 WAIT_FCU→
  WAIT_NAV_STABLE 应自然推进（SIH 的 GPS/IMU 由仿真提供，比真 GPS 收敛快）；
- 解锁 → 切 GUIDED（飞手确认步骤照真机流程走，本身就是彩排）。

## 4. 跑任务（两档，先 A 后 B）

**档 A：仅舵机（SIM_OH_MASK=768）**

1. Jetson 起 mission 状态机 + 真值感知替身（契约 v1.3），或 MAVProxy 手动推进；
2. 任务跑到 RELEASE：`SERVO_OUTPUT_RAW` 回读 SERVO9/10 应逐拍出现
   1100（收拢）→1900（释放）时序——对照 `servo_test.py` 台架基线；
3. 全 18 态推进到 DONE 无卡死 = 主验收过。

**档 B：+ 电机（SIM_OH_MASK=783，改参后重启）**

1. 确认拆桨、电调电源（电池）在位；
2. 重跑任务：电机跟随油门/姿态输出；听堵转/异响，量舵机供电电压是否被拉垮
   （UBEC 到位后此档同时验证供电方案）；
3. GCS 看仿真位置沿航线推进，对照 §5.5 时序预算表。

## 5. 恢复（用完必做）

1. 重刷 `02_飞控与硬件/firmware/arducopter_beta_CUAV-V6X-v2.apj`（锁定 beta）；
2. 重插 USB → 导入 §1 的 params-export 备份 → 重启 →
   `verify_params.py --diff <备份文件>` 干净；
3. `check_fc.py` 对照版本号；校准状态无异常不动（刷同板固件不清校准，
   但 FORMAT_VERSION=0 玩过就必须全重校）。

## 6. 常见问题

| 症状 | 排查 |
|---|---|
| 解锁被拒 RC 缺失 | 真遥控器遥控链路（09-12 定案 CH5/CH6 开关），或 MAVLink `rc override` |
| 舵机不动 | SIM_OH_MASK 位对不对（bit8/9=ch9/10）→ SERVO9/10_FUNCTION=0 → 舵机供电（UBEC/台架替代电源） |
| 电机不转 | 电调是否上电（OPTO 无 BEC 需电池）；解锁油门 MOT_SPIN_MIN；DShot/PWM 协议与真电调匹配 |
| EKF 不就绪 | 回读 AHRS_EKF_TYPE/GPS1_TYPE/SIM_MAG1_DEVID 三件是否落地 |
| GCS 地图在堪培拉 | SIM_OPOS_* 未设或未重启，正常现象不影响任务 |
| 参数写不进/写后丢 | 09-12 铁律：写参数等 5 秒再重启 |
