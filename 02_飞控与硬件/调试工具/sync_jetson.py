# -*- coding: utf-8 -*-
"""Jetson 一键同步：仓库(SSOT) → Jetson 单向推送 + 哈希核验（替代人肉 scp 单文件）

背景（2026-09-19 定案）：Jetson 上没有本仓 git 克隆（cuadc_ws 为手工拷贝，~ 下 .git 全是
第三方仓），同步曾靠零散 scp，静默漂移实锤：09-17 重构的注释路径、SIH 脚本 MultiCopter
大小写勘误、9 个真视觉线文件、rescue_sim 整包 CRLF 污染——全部只在仓库侧。
方向纪律：**仓库是唯一真源，只推不拉**；Jetson 侧若有独有改动，应先人肉带回仓库提交，
再用本工具下发（REMOTE_ONLY 报告就是这类漏网提示）。

用法（ops.py `sync` 命令入口）:
  python sync_jetson.py              # check（默认）：全清单 md5 比对，只读报告
  python sync_jetson.py push         # 推送全部 不同+仓库独有 文件（逐文件确认）
  python sync_jetson.py push --yes   # 同上，免确认
  python sync_jetson.py push 关键词 [关键词...]   # 只推路径含关键词的文件（--yes 可混用）
  python sync_jetson.py diff 关键词  # 看某文件 仓库vs Jetson 统一 diff（取第一个匹配）
  python sync_jetson.py manifest [development|candidate|approved_release] [--note "..."] [--push]
                                   # D3 冻结证据链：HEAD+脏状态+全清单 md5 → 参数备份/manifest_*.json（--push 加推 Jetson ~）

映射清单（改布局必须同步改这里 + 根 README + SSOT §0）:
  03_机载软件/cuadc_{interfaces,mission,perception}  → jetson:cuadc_ws/src/同名
  04_仿真/仿真环境/cuadc_rescue_sim                  → jetson:cuadc_ws/src/cuadc_rescue_sim
  02_飞控与硬件/调试工具/{11 个 Jetson 侧脚本}        → jetson:~/
排除：__pycache__、*.pyc、*.exe、*.pdb（Windows 构建产物）。
换行符：仓库 .gitattributes 钉死 LF（代码要上 Jetson）；Jetson 侧 CRLF 残留会判 DIFF，
push 即以仓库 LF 版覆盖——这正是 09-19 rescue_sim 整包 CRLF 污染的修法。
对运行中服务的影响：感知 systemd 服务跑 install/ 空间，推 src/ 不影响运行；
新入口点/改代码要生效需在 Jetson 重新 colcon build（本工具不代做，防误触发）。
"""
import hashlib
import subprocess
import sys
from pathlib import Path

HOST = "jetson"  # ~/.ssh/config 别名（换网只改 HostName，工具不改）
REPO = Path(__file__).resolve().parents[2]
TOOLS = Path(__file__).resolve().parent

TREE_MAPS = [
    ("03_机载软件/cuadc_interfaces", "cuadc_ws/src/cuadc_interfaces"),
    ("03_机载软件/cuadc_mission", "cuadc_ws/src/cuadc_mission"),
    ("03_机载软件/cuadc_perception", "cuadc_ws/src/cuadc_perception"),
    ("04_仿真/仿真环境/cuadc_rescue_sim", "cuadc_ws/src/cuadc_rescue_sim"),
]
LOOSE_FILES = ["fc_dryrun.sh", "fc_armrun.sh", "fc_armtest.sh", "fc_servo_test.sh",
               "fc_servo_diag.sh", "fc_log_pull.py", "sih_jetson_build.sh",
               "bench_logger.py", "bench_drop.py", "gcs_pump.py",
               "recon_bench_onekey.sh"]
EXCL_DIRS = {"__pycache__"}
EXCL_SUFFIX = (".pyc", ".exe", ".pdb")


def md5_file(p: Path) -> str:
    h = hashlib.md5()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def sh_quote(s: str) -> str:
    return "'" + s.replace("'", "'\\''") + "'"


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", **kw)


def local_manifest():
    """{jetson相对路径: (本地绝对路径, md5)}；LOOSE_FILES 缺失即报错（清单与库必须一致）"""
    items = {}
    for local_dir, remote_dir in TREE_MAPS:
        root = REPO / local_dir
        for p in sorted(root.rglob("*")):
            if p.is_dir():
                continue
            if EXCL_DIRS & set(p.parts):
                continue
            if p.suffix.lower() in EXCL_SUFFIX:
                continue
            rel = p.relative_to(root).as_posix()
            items[f"{remote_dir}/{rel}"] = (p, md5_file(p))
    for name in LOOSE_FILES:
        p = TOOLS / name
        if not p.exists():
            sys.exit(f"ERROR: 清单文件 {name} 不在 {TOOLS}（清单与仓库不同步，先查）")
        items[name] = (p, md5_file(p))
    return items


def remote_manifest(remote_paths):
    """一次 ssh 批量 md5sum；缺文件从 stderr 识别 → 返回 {path: md5|None}"""
    script = "cd ~ && md5sum " + " ".join(sh_quote(p) for p in remote_paths)
    r = run(["ssh", "-o", "BatchMode=yes", HOST, script])
    hashes = {}
    for line in (r.stdout or "").splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and len(parts[0]) == 32:
            hashes[parts[1].lstrip("*")] = parts[0]
    for p in remote_paths:
        if p not in hashes:
            hashes[p] = None  # md5sum 对缺失文件只吐 stderr
    return hashes


def cmd_manifest(args):
    """D3（SSOT §15）：比赛冻结证据链——HEAD + 脏状态 + 全清单 md5 → manifest JSON
    落 02_飞控与硬件/参数备份/manifest_<UTC时间>_<级别>.json；--push 加推 Jetson ~。
    版本三级：development（平时）/ candidate（台架联测冻结）/ approved_release（比赛前）。
    权重 SHA256 不在此清单（已有独立三端归档机制，见 SSOT §12 P0.2）。"""
    import json
    from datetime import datetime, timezone

    level, note, push = "candidate", "", "--push" in args
    positional = []
    it = iter(args)
    for a in it:
        if a == "--level":
            level = next(it, level)
        elif a == "--note":
            note = next(it, note)
        elif a == "--push":
            push = True
        elif not a.startswith("--"):
            positional.append(a)          # 位置参数：第一个=level（同步 Jetson 用法习惯）
    if positional:
        level = positional[0]
    if level not in ("development", "candidate", "approved_release"):
        sys.exit(f"ERROR: level 须为 development/candidate/approved_release，得到 {level!r}")

    head = run(["git", "-C", str(REPO), "rev-parse", "HEAD"]).stdout.strip()
    dirty = run(["git", "-C", str(REPO), "status", "--porcelain"]).stdout.splitlines()
    local = local_manifest()
    files = [{"jetson_path": rp, "md5": lh}
             for rp, (_lp, lh) in sorted(local.items())]
    now_utc = datetime.now(timezone.utc)
    out = (REPO / "02_飞控与硬件" / "参数备份" /
           f"manifest_{now_utc.strftime('%Y%m%d_%H%M%S')}_{level}.json")
    doc = {
        "level": level,
        "note": note,
        "generated_at_utc": now_utc.isoformat(timespec="seconds"),
        "git_commit": head,
        "git_dirty_entries": len(dirty),
        "file_count": len(files),
        "files": files,
    }
    out.write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"[manifest] {level} → {out.name}")
    print(f"  commit={head[:12]}  files={len(files)}  脏条目={len(dirty)}")
    for line in dirty[:10]:
        print(f"  dirty: {line}")
    if dirty:
        print("  ⚠️ 工作区不干净——冻结前应先全部提交，再重新生成 manifest")
    if push:
        r = run(["scp", "-o", "BatchMode=yes", str(out),
                 f"{HOST}:~/cuadc_manifest_{level}.json"])
        print("[manifest] 推送 Jetson: " + ("OK" if r.returncode == 0 else "失败"))


def main():
    args = [a for a in sys.argv[1:]]
    mode = args[0] if args else "check"
    yes = "--yes" in args
    keys = [a for a in args[1:] if a != "--yes"]
    if mode not in ("check", "push", "diff", "manifest"):
        print(__doc__)
        sys.exit(1)
    if mode == "manifest":
        sys.exit(cmd_manifest(keys))

    local = local_manifest()
    remote = remote_manifest(sorted(local))
    diff, only_local, ok = [], [], 0
    for rpath, (lpath, lhash) in local.items():
        rhash = remote.get(rpath)
        if rhash is None:
            diff.append((rpath, "缺失"))
        elif rhash != lhash:
            diff.append((rpath, "不同"))
        else:
            ok += 1
    only_remote = [p for p in sorted(remote) if p not in local]

    print(f"[check] 清单 {len(local)} 项：一致 {ok} / 不同或缺失 {len(diff)}"
          f" / Jetson 多出 {len(only_remote)}")
    for p, st in diff:
        print(f"  {st}  {p}")
    for p in only_remote:
        print(f"  ⚠️ Jetson 独有(仓库没有，不自动删): {p}")
    if mode == "check" or not diff:
        if mode == "push" and not diff:
            print("[push] 无可推项，结束")
        sys.exit(0)

    if mode == "diff":
        key = keys[0] if keys else diff[0][0]
        target = next((p for p, _ in diff if key in p), None)
        if not target:
            sys.exit(f"ERROR: 差异清单里没有匹配 {key!r} 的文件")
        r = run(["ssh", "-o", "BatchMode=yes", HOST, f"cat {sh_quote(target)}"])
        import difflib
        local_lines = local[target][0].read_bytes().decode("utf-8", "replace").splitlines()
        remote_lines = (r.stdout or "").replace("\r\n", "\n").splitlines()
        for line in list(difflib.unified_diff(remote_lines, local_lines,
                                              "jetson", "repo(SSOT)", lineterm=""))[:40]:
            print(line)
        sys.exit(0)

    # ---- push ----
    if keys:
        picked = [(p, st) for p, st in diff if any(k in p for k in keys)]
        if not picked:
            sys.exit(f"ERROR: 关键词 {keys} 没匹配到任何待推文件")
    else:
        picked = diff
    print(f"\n[push] 待推 {len(picked)} 项" + ("（--yes 直推）" if yes else "，逐项确认："))
    approved = []
    for p, st in picked:
        if yes:
            approved.append(p)
            continue
        a = input(f"  推 {p} ? [y/N/全部] ").strip().lower()
        if a in ("y", "yes"):
            approved.append(p)
        elif a in ("a", "all"):
            approved.extend(x for x, _ in picked[picked.index((p, st)):])
            break
    if not approved:
        print("[push] 未推任何文件")
        sys.exit(0)

    # 按目标目录分组，Python tarfile 流 → ssh 远端 GNU tar 解包（免 scp 空格/中文/SFTP
    # 版本坑，也不依赖本地 tar 的 Windows 路径行为）
    groups = {}
    for rpath in approved:
        if "/" in rpath:
            for local_dir, remote_dir in TREE_MAPS:
                if rpath.startswith(remote_dir + "/"):
                    groups.setdefault((str(REPO / local_dir), remote_dir), []).append(
                        rpath[len(remote_dir) + 1:])
                    break
        else:
            groups.setdefault((str(TOOLS), "~"), []).append(rpath)
    import tarfile
    for (lroot, rdir), rels in groups.items():
        rdir_arg = '"$HOME"' if rdir == "~" else sh_quote(rdir)
        mkdir = "" if rdir == "~" else f"mkdir -p {sh_quote(rdir)} && "
        ssh = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", HOST,
             f"{mkdir}tar -C {rdir_arg} -xf -"],
            stdin=subprocess.PIPE)
        try:
            with tarfile.open(fileobj=ssh.stdin, mode="w|") as tf:
                for rel in rels:
                    tf.add(str(Path(lroot) / rel), arcname=rel)
            ssh.stdin.close()
        except BrokenPipeError:
            pass
        ssh.wait()
        if ssh.returncode:
            sys.exit(f"ERROR: 推送 {rdir} 失败（ssh 返回 {ssh.returncode}）")
        print(f"  → {rdir}  {len(rels)} 文件", flush=True)

    # 推后全量复核
    remote2 = remote_manifest(sorted(local))
    bad = [p for p in approved
           if remote2.get(p) != local[p][1]]
    if bad:
        sys.exit("ERROR: 推后复核不一致:\n  " + "\n  ".join(bad))
    print(f"[push] 完成：{len(approved)} 项已推并复核一致。"
          "提醒：感知服务跑 install/ 空间不受影响；要生效需 Jetson 重新 colcon build。")


if __name__ == "__main__":
    main()
