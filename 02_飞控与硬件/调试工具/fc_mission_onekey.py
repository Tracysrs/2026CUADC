#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务全流程一键启动（Jetson 侧）：冷启动自检 → MAVROS → 任务状态机 → 看护 → 结果摘要

对应 SSOT §5.1 全生命周期 18 态（WAIT_FCU→…→DONE）。投放方案对照
《多旋翼无人机侦察与救援规则2026》3.1.2（投放区 3 筒各一：1号=15cm/500分、
2号=20cm/300分、3号=25cm/100分）与 6.1.2（两个载荷投中同一 A 区只算一次有效）。

方案（位置参数 <plan>，规则 3.1.2 筒号即直径：1号=15cm/500分、2号=20cm/300分、3号=25cm/100分）:
  23  3号+2号  25cm(100) + 20cm(300) = 400 分   drop_order=pair_23（旧名 conservative，先大保底，默认）
  13  1号+3号  15cm(500) + 25cm(100) = 600 分   drop_order=pair_13（跳过中筒）
  12  1号+2号  15cm(500) + 20cm(300) = 800 分   drop_order=pair_12（旧名 aggressive，冲奖）
没有「中+中」：规则 3.1.2 投放区每规格仅一筒，6.1.2 双载荷投同一 A 区只算一次（封顶 300 分
比 23 档还低），且本机 drop_logic 已投筒拉黑（0.25m/120s）机制上禁止同筒二次投放。

用法（Jetson 上，非交互 ssh 直用亦可，缺 ROS 环境自动 source 后重执行自身）:
  python3 fc_mission_onekey.py <23|13|12> [选项]
  python3 fc_mission_onekey.py 12 --no-vision     # M1 无视觉演练口径
  python3 fc_mission_onekey.py 12 --dry-run       # 只打印将执行的链路，不碰系统
PC 端（Windows）入口 = ops 自动 ssh 远跑，等价于上机执行:
  python ops.py onekey <23|13|12> [选项]

选项:
  --no-vision       m1_no_vision_mode:=true（无视觉演练；⚠️ 该模式飞预设航线，无搜索/对准/投放，投放方案不生效）
  --auto-arm        auto_arm_on_guided:=true（默认关 = 解锁权在飞手，仓库红线 2）
  --live-drop       enable_release_output:=true 实弹投放（默认 false = 舵机干跑只打日志；上场实投必须显式给）
  --fcu URL         覆盖 FCU 链路（默认 ttyTHS1:921600 → 不在则 cuadc-fc:115200 自动降级）
  --timeout N       看护总秒数（默认 300 = 比赛时间上限 5 分钟）
  --skip-services   跳过 systemd 冷启动自检（默认查 cuadc-perception 服务）
  --dry-run         只打印链路与参数后退出

安全设计:
  · 第一次 Ctrl-C = 只退看护，任务节点与 MAVROS 保持运行（飞机可能仍在自主飞！），
    打印手工收尾命令；第二次 Ctrl-C 或确认落地后才强杀全部子进程。
  · 正式模式（未给 --no-vision）下感知服务不活跃 → 拒启（视觉起飞门禁 ≥3 帧会卡死，早失败早好）。

方案 2（中+中）被否的历史结论（2026-09-29 按规则 PDF 查证，详见上方案表）:
  ① 规则 3.1.2 投放区仅 3 筒各一，场上只有一个 2号筒；
  ② 规则 6.1.2 双载荷投同一 A 区「视为一次有效」= 封顶 300 分；
  ③ 本机 drop_logic 已投筒拉黑（0.25m/120s）机制上禁止同筒二次投放。
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

LOGDIR_BASE = Path("/tmp/cuadc_mission_onekey")
SERVICES = ["cuadc-perception.service"]
TERMINAL_STATES = {"DONE", "ABORT", "PILOT_OVERRIDE"}
PLAN_INFO = {
    "23": ("pair_23", "3号筒(25cm,100) + 2号筒(20cm,300) = 400 分（先大保底）"),
    "13": ("pair_13", "1号筒(15cm,500) + 3号筒(25cm,100) = 600 分（跳过中筒）"),
    "12": ("pair_12", "1号筒(15cm,500) + 2号筒(20cm,300) = 800 分（冲奖）"),
}


def sh(cmd, timeout=25):
    """跑一条命令，stdout+stderr 合并返回（失败返回空串不抛）。"""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=timeout)
        return (r.stdout or "") + (r.stderr or "")
    except (subprocess.TimeoutExpired, OSError) as e:
        return f"[sh 超时/失败: {e}]"


def topic_once(topic, typ, timeout=8):
    """ros2 topic echo --once 的一次快照（原始文本）。"""
    return sh(["timeout", str(timeout), "ros2", "topic", "echo", "--once", topic, typ])


def ensure_ros(argv):
    """ros2 不在 PATH → bash -lc source 后重执行自身（kacha 同款，非交互 ssh 直用）。"""
    if shutil.which("ros2"):
        return
    print("[env] ros2 不在 PATH，source ROS 后重执行自身…")
    src = ("source /opt/ros/humble/setup.bash && "
           "source $HOME/cuadc_ws/install/setup.bash")
    cmd = " ".join("'" + a.replace("'", "'\\''") + "'" for a in [sys.executable] + argv)
    os.execvp("bash", ["bash", "-lc", f"{src} && exec {cmd}"])


def pick_fcu(override):
    """FCU 设备选择，与 fc_dryrun.sh 同序：env 覆盖 > TELEM3 主链 > USB 备份。"""
    if override:
        return override
    if os.environ.get("FCU_URL"):
        return os.environ["FCU_URL"]
    if Path("/dev/ttyTHS1").exists():
        return "/dev/ttyTHS1:921600"
    if Path("/dev/cuadc-fc").exists():
        return "/dev/cuadc-fc:115200"
    return None


def preflight(skip_services, no_vision):
    """[0] 冷启动自检：systemd 自启服务 / 磁盘余量 / 工作空间可执行。返回是否通过。"""
    ok = True
    if not skip_services:
        for svc in SERVICES:
            state = sh(["systemctl", "is-active", svc], timeout=10).strip()
            if state != "active":
                print(f"  ⏳ {svc} = {state or '未知'}，等 30s（冷启动自启验证，上场检查表项）")
                for _ in range(15):
                    time.sleep(2)
                    if sh(["systemctl", "is-active", svc], timeout=10).strip() == "active":
                        state = "active"
                        break
            if state == "active":
                print(f"  ✅ {svc} active")
            else:
                if no_vision:
                    print(f"  ⚠️ {svc} 未就绪——--no-vision 模式不依赖感知，继续")
                else:
                    print(f"  ❌ {svc} 未就绪且为正式模式（视觉起飞门禁会卡死）。"
                          f"先查感知自启，或确认无视觉演练时显式给 --no-vision")
                    ok = False
    free_mb = shutil.disk_usage("/").free // (1 << 20)
    print(f"  {'✅' if free_mb > 512 else '❌'} 磁盘剩余 {free_mb}MB（SSOT §5.3 红线 512MB）")
    if free_mb <= 512:
        ok = False
    node_bin = Path.home() / "cuadc_ws/install/cuadc_mission/lib/cuadc_mission/cuadc_mission_node"
    if node_bin.exists():
        print("  ✅ cuadc_mission_node 已编译（install 空间）")
    else:
        print(f"  ❌ 未找到 {node_bin}——先 colcon build")
        ok = False
    return ok


def snapshot_line():
    """看护周期快照：模式/解锁/电池一行（读不到就显示 ?）。"""
    st = topic_once("/mavros/state", "mavros_msgs/msg/State")
    mode = "?" if "mode: ''" in st else next(
        (l.split(":")[1].strip() for l in st.splitlines() if l.startswith("mode:")), "?")
    armed = next((l.split(":")[1].strip() for l in st.splitlines()
                  if l.startswith("armed:")), "?")
    bat = topic_once("/mavros/battery", "sensor_msgs/msg/BatteryState")
    volt = next((l.split(":")[1].strip() for l in bat.splitlines()
                 if l.startswith("voltage:")), "?")
    return f"mode={mode} armed={armed} bat={volt}V"


def last_state(mission_log):
    """从 mission.log 抓最新状态：返回 (STATE 名或 None, 是否见到任务结束行)。"""
    try:
        text = mission_log.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None, False
    states = [l.split("STATE ->", 1)[1].strip() for l in text.splitlines()
              if "STATE ->" in l]
    return (states[-1] if states else None), ("任务结束" in text)


def kill_procs(procs):
    for p in procs:
        try:
            p.terminate()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(add_help=True, description="任务全流程一键启动")
    ap.add_argument("plan", choices=["23", "13", "12"],
                    help="投放两筒组合（规则 3.1.2 筒号即直径）：23=400分保底 13=600分跳中筒 12=800分冲奖")
    ap.add_argument("--no-vision", action="store_true")
    ap.add_argument("--auto-arm", action="store_true")
    ap.add_argument("--live-drop", action="store_true")
    ap.add_argument("--fcu", default=None)
    ap.add_argument("--timeout", type=int, default=300)
    ap.add_argument("--skip-services", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    drop_order, plan_desc = PLAN_INFO[args.plan]
    fcu = pick_fcu(args.fcu)
    overrides = [f"drop_order:={drop_order}",
                 f"m1_no_vision_mode:={'true' if args.no_vision else 'false'}",
                 f"auto_arm_on_guided:={'true' if args.auto_arm else 'false'}",
                 f"enable_release_output:={'true' if args.live_drop else 'false'}"]

    print("=" * 64)
    print(f"方案 {args.plan}: {plan_desc}")
    print(f"参数注入: {' '.join(overrides)}")
    print(f"FCU: {fcu or '（未找到任何设备！）'}")
    print(f"看护: {args.timeout}s | 日志: {LOGDIR_BASE}_<时间戳>/")
    if args.no_vision:
        print("⚠️ --no-vision 预设航线演练：无搜索/对准/投放，drop_order 不生效")
    if args.live_drop:
        print("⚠️⚠️ --live-drop 实弹投放已开：确认瓶在位、人员远离、安全员就位！")
    if not args.auto_arm:
        print("解锁权在飞手（红线 2）：等飞手切 GUIDED → 手动解锁")
    print("=" * 64)
    if args.dry_run:
        print("[dry-run] 链路: 冷启动自检 → 清陈旧进程 → mavros+gcs_pump → 等 connected")
        print("          → 任务节点(参数注入如上) → 状态看护 → 终态收尾+摘要")
        print("[dry-run] 结束，未触碰系统")
        return

    ensure_ros(sys.argv)
    if not fcu:
        sys.exit("❌ 未找到 FCU 设备（ttyTHS1 与 cuadc-fc 均不在），可用 --fcu 强制指定")
    if not preflight(args.skip_services, args.no_vision):
        sys.exit("❌ 冷启动自检未过（上方 ❌ 项），拒绝进入任务链")

    # ---- [1] 日志目录 + 清陈旧进程（上次会话泄漏的 mavros 会抢串口/抢 DDS）----
    logdir = Path(f"{LOGDIR_BASE}_{datetime.now().strftime('%m%d_%H%M%S')}")
    logdir.mkdir(parents=True, exist_ok=True)
    sh(["ros2", "daemon", "stop"])
    sh(["pkill", "-f", "cuadc_mission_[n]ode"])
    sh(["pkill", "-f", "[m]avros_node"])
    sh(["pkill", "-f", "[g]cs_pump.py"])
    time.sleep(1)

    # ---- [2] MAVROS + GCS 心跳泵 ----
    print(f"[2] 起 mavros（{fcu}）+ gcs_pump")
    mlog = (logdir / "mavros.log").open("w")
    mavros = subprocess.Popen(
        ["ros2", "launch", "mavros", "apm.launch",
         f"fcu_url:={fcu}", "gcs_url:=tcp-l://0.0.0.0:14550"],
        stdout=mlog, stderr=subprocess.STDOUT)
    pump = subprocess.Popen(["python3", str(Path.home() / "gcs_pump.py")],
                            stdout=(logdir / "pump.log").open("w"),
                            stderr=subprocess.STDOUT)

    # ---- [3] 等连接（30s）----
    print("[3] 等飞控连接（30s）")
    connected = False
    for _ in range(15):
        st = topic_once("/mavros/state", "mavros_msgs/msg/State")
        if "connected: true" in st:
            connected = True
            break
        time.sleep(2)
    if not connected:
        kill_procs([mavros, pump])
        sys.exit("❌ 30s 内飞控未 connected——查链路（03 册 §1）或用 fc_dryrun.sh 分诊")
    print(f"  ✅ 已连接 | {snapshot_line()}")

    # ---- [4] 任务节点（正式模式即比赛口径）+ 状态轨迹记录 ----
    print(f"[4] 起任务节点（drop_order={drop_order}）")
    mmission = (logdir / "mission.log").open("w")
    mission = subprocess.Popen(
        ["ros2", "run", "cuadc_mission", "cuadc_mission_node", "--ros-args",
         "--params-file", str(Path.home() / "cuadc_ws/src/cuadc_mission/config/mission_params.yaml"),
         "-p", overrides[0], "-p", overrides[1], "-p", overrides[2], "-p", overrides[3]],
        stdout=mmission, stderr=subprocess.STDOUT)
    states_f = (logdir / "mission_states.txt").open("w")
    states_rec = subprocess.Popen(
        ["ros2", "topic", "echo", "/cuadc/mission_state"],
        stdout=states_f, stderr=subprocess.DEVNULL)
    procs = [mission, states_rec, pump, mavros]
    mmission.flush()
    mission_log = logdir / "mission.log"

    print(">>> 飞手动作：等 LOCK_FRAME 锁好后，遥控切 GUIDED（WAIT_GUIDED 扳机）"
          + ("，程序将自动解锁" if args.auto_arm else "，再手动解锁"))
    print(f"[5] 看护中（最长 {args.timeout}s，Ctrl-C 一次=退看护不杀任务）…")

    # ---- [5] 看护循环 ----
    seen_flags = {}   # 一次性提示去重（guided/arm）+ 最近状态缓存（"_last"）
    t0 = time.time()
    result, code = "超时", 1
    try:
        while time.time() - t0 < args.timeout:
            time.sleep(5)
            state, ended = last_state(mission_log)
            if state and state != seen_flags.get("_last"):
                seen_flags["_last"] = state
                print(f"  t={int(time.time()-t0)}s → {state} | {snapshot_line()}")
            if state == "WAIT_GUIDED" and "guided" not in seen_flags:
                seen_flags["guided"] = 1
                print(">>> 飞手动作：现在切 GUIDED")
            if state == "WAIT_ARM" and not args.auto_arm and "arm" not in seen_flags:
                seen_flags["arm"] = 1
                print(">>> 飞手动作：保持 GUIDED 并解锁")
            if ended or (state in TERMINAL_STATES):
                result, code = (state or "DONE"), (0 if state == "DONE" else 1)
                break
    except KeyboardInterrupt:
        print("\n⚠️ 看护退出（任务节点/MAVROS 仍在运行——飞机可能还在飞！）")
        print(f"   手工跟踪: tail -f {mission_log}")
        print("   手工收尾: pkill -f cuadc_mission_[n]ode && pkill -f '[m]avros_node'")
        states_rec.terminate()
        sys.exit(130)

    # ---- [6] 收尾 + 摘要 ----
    time.sleep(2)
    kill_procs(procs)
    sh(["pkill", "-f", "cuadc_mission_[n]ode"])
    tail = sh(["grep", "-aE", "任务结束|弃桶|失败|ABORT|释放|舵机", str(mission_log)])[-1500:]
    traj = sh(["grep", "-oE", r"[A-Z_]{4,}", str(logdir / "mission_states.txt")])
    seen = []
    for tok in traj.split():
        if not seen or seen[-1] != tok:
            seen.append(tok)
    print("=" * 64)
    print(f"结果: {result}（退出码 {code}） | 方案 {args.plan} = {plan_desc}")
    print(f"状态轨迹: {' → '.join(seen) if seen else '（未录到）'}")
    if tail:
        print("--- mission.log 关键行（尾 1500 字）---")
        print(tail)
    print(f"日志: {logdir}/")
    sys.exit(code)


if __name__ == "__main__":
    main()
