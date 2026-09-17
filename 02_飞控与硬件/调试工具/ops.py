# -*- coding: utf-8 -*-
"""现场操作统一入口：按《现场操作_上电到遥控手动飞行实施步骤.md》的阶段分类调用调试脚本。

用法（在 02_飞控与硬件/调试工具/ 目录下）：
  python ops.py              # 打印分阶段命令菜单
  python ops.py <命令> [参数...]   # 参数原样透传给对应脚本，如：
  python ops.py fc
  python ops.py gps
  python ops.py geo here          # = python site_geo_check.py here
  python ops.py motor COM5 3      # 拆桨状态才允许！只转 M3
  python ops.py params-diff ../V6X_ardupilot_params.param
  python ops.py log list          # Jetson 上用

约定：
  .py 用当前解释器运行；.sh 是 Jetson 专用（bash 运行），Windows 下会提示。
  跑脚本前先断开 Mission Planner（占串口）；COM 号重启会变，透传参数里带上。
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# 命令 -> (脚本文件, 所属阶段, 说明)
CMDS = {
    # ---- 阶段 0：解锁排障 ----
    "fc":            ("check_fc.py",    "阶段0", "连飞控读版本/模式/电池/姿态（定位 PreArm 报错）"),
    "prearm":        ("prearm_diag.py", "阶段0", "解锁前诊断：模式/RC/电池/EKF/罗盘一致性一次看全（只读）"),
    "params-diff":   ("verify_params.py", "阶段0/2", "参数一致性核对，用法: params-diff <参数文件>"),
    "params-export": ("verify_params.py", "阶段0", "参数全量备份到 02_飞控与硬件/参数备份/"),
    "read-params":   ("read_params.py", "阶段0", "读机架/串口等指定参数"),
    "lazy-params":   ("activate_lazy_params.py", "阶段0", "激活懒加载参数组并重启（4.7-beta 特性）"),
    # ---- 阶段 1/2：上电检查 ----
    "gps":           ("check_gps.py",   "阶段2", "GPS 验收（3D Fix + ≥8星 + HDOP<1.5）"),
    "radio":         ("check_radio.py", "阶段2", "数传诊断（默认 COM10，只读；严禁 ATI5）"),
    "geo":           ("site_geo_check.py", "阶段2", "场地地理/就地测试：probe | here | record N"),
    "rtk":           ("rtk_setup.py",  "阶段2", "RTK rover 一键配置+验证（GPS2_TYPE=3 + SERIAL4_BAUD=921）"),
    # ---- 阶段 3：动力 ----
    "motor":         ("motor_test.py",  "阶段3", "电机顺序/方向（拆桨！）: motor [COM口] [输出]"),
    "servo":         ("servo_test.py",  "阶段3", "舵机投放测试（COM 直连，回读 PWM 分辨供电问题）: servo [COM口] [9|10|9,10]"),
    # ---- Jetson 专用（.sh / mavftp）----
    "dryrun":        ("fc_dryrun.sh",   "Jetson", "任务状态机干跑（无桨不解锁）: dryrun [秒数]"),
    "armrun":        ("fc_armrun.sh",   "Jetson", "解锁试跑·状态机路径（无 GPS 卡 WAIT_NAV_STABLE）"),
    "armtest":       ("fc_armtest.sh",  "Jetson", "解锁试跑·直接路径（CommandBool 10s）"),
    "servo-test":    ("fc_servo_test.sh", "Jetson", "舵机投放时序（收1100/释1900）"),
    "servo-diag":    ("fc_servo_diag.sh", "Jetson", "舵机不动时诊断 /mavros/rc/out"),
    "log":           ("fc_log_pull.py", "Jetson", "拉 dataflash 日志: log list | log get 名字"),
}

MENU_ORDER = ["阶段0", "阶段0/2", "阶段2", "阶段3", "Jetson"]


def menu():
    print("现场操作命令菜单（对应 05_操作与比赛/现场操作_上电到遥控手动飞行实施步骤.md）\n")
    for stage in MENU_ORDER:
        print(f"── {stage} " + "─" * 40)
        for name, (script, st, desc) in CMDS.items():
            if st != stage:
                continue
            print(f"  {name:<15} {desc}   [{script}]")
        print()
    print("用法: python ops.py <命令> [参数...]   例: python ops.py geo here")
    print("注意: 跑前断开 Mission Planner；motor/servo 类命令必须拆桨状态。")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        menu()
        return
    name, *args = sys.argv[1:]
    if name not in CMDS:
        print(f"未知命令: {name}。可用命令见: python ops.py\n")
        menu()
        sys.exit(1)
    script, stage, desc = CMDS[name]
    path = os.path.join(HERE, script)
    if not os.path.exists(path):
        sys.exit(f"脚本不存在: {path}")
    if script.endswith(".sh"):
        if os.name == "nt":
            print(f"[{stage}] {desc}")
            print("该脚本是 Jetson 专用（依赖 /dev 设备与 mavros 环境），Windows 下无法直接运行。")
            print("在 Jetson 上执行: bash " + script + " " + " ".join(args))
            sys.exit(2)
        cmd = ["bash", path] + args
    else:
        cmd = [sys.executable, path] + args
    print(f">>> [{stage}] {desc}")
    print(f">>> {' '.join(cmd)}\n")
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
