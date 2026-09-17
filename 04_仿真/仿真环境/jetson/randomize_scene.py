#!/usr/bin/env python3
"""随机布景：在任务区域内随机摆放投放桶/侦察桶/危化品标识，并联动三处真值。

做什么（一次跑完）：
  1. 随机生成 3 投放桶（投放区 x 27.5~32.5 / y ±4，两两间距 ≥1.5m）
     与 5 侦察桶 + 3 张危化品标识（侦察区 x 52.5~57.5 / y ±4，间距 ≥1.2m）；
  2. 改写飞行世界 ~/sim_scripts/cuadc_rescue_flight.sdf 的桶阵/标识段
     （只动 drop/recon/hazard 块，Sensors/光照/出生位姿等一律不碰）；
  3. 重写 generated_scene.yaml 真值（真值感知替身 scene_truth_perception 读它）；
  4. 写 ~/sim_scripts/judge_truth_params.yaml（虚拟判定节点按新桶位判分）；
  5. --restart-gz（默认开）：只重启 gz 服务端载入新场景——SITL 不动，
     不进 15~20min 不稳定窗，飞机回出生坪。

之后正常跑：fcu_ready.py → fc_sitl_m3.sh（真值替身）/ fc_sitl_m2.sh（CV 真感知，
  CV 从相机看新场景，无需真值；judge 走新参数）。

用法：python3 randomize_scene.py [--seed N]（缺省=随机种子）
"""

import argparse
import random
import re

DROP_MODELS = [  # (模型, 直径m, 分值) —— 与 SSOT 计分一致
    ('drop_bucket_15', 0.15, 500),
    ('drop_bucket_20', 0.20, 300),
    ('drop_bucket_25', 0.25, 100),
]
RECON_N = 5
MARKER_N = 3
# 10 类危化品纹理（models/hazard_marker/materials/textures/，对应附件 11 类别）
MARKERS = ['explosive', 'flammable', 'nonflammable_gas', 'corrosive', 'irritant',
           'toxic', 'dangerous_when_wet', 'spontaneously_combustible',
           'radioactive', 'biohazard']
DROP_ZONE = (27.5, 32.5, 4.0)    # x0, x1, 半宽（中心 30，5m×8m）
RECON_ZONE = (52.5, 57.5, 4.0)   # 中心 55
MARGIN = 0.4                     # 离区边最小距离
DROP_MIN_GAP = 1.5
RECON_MIN_GAP = 1.2


def sample_points(n, zone, min_gap, rng):
    x0, x1, hw = zone
    pts = []
    tries = 0
    while len(pts) < n:
        tries += 1
        if tries > 5000:
            raise RuntimeError('布点失败：区太小/间距太密')
        p = (rng.uniform(x0 + MARGIN, x1 - MARGIN), rng.uniform(-hw + MARGIN, hw - MARGIN))
        if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= min_gap ** 2 for q in pts):
            pts.append(p)
    return pts


def build_scene_block(drop_pts, recon_pts, markers, rng):
    """生成飞行世界的 drop/recon include 段 + 危化品标识模型段。"""
    parts = []
    for i, ((x, y), (model, _d, _s)) in enumerate(zip(drop_pts, DROP_MODELS), 1):
        parts.append(
            '<include>\n  <name>drop_%d</name>\n  <uri>model://%s</uri>\n'
            '  <pose>%.3f %.3f 0 0 0 0.0</pose>\n</include>\n' % (i, model, x, y))
    for i, (x, y) in enumerate(recon_pts, 1):
        parts.append(
            '<include>\n  <name>recon_%d</name>\n  <uri>model://recon_bucket</uri>\n'
            '  <pose>%.3f %.3f 0 0 0 0.0</pose>\n</include>\n' % (i, x, y))
    for marker, (x, y), i in zip(markers, recon_pts, range(1, RECON_N + 1)):
        if marker is None:
            continue
        yaw = rng.uniform(0, 6.283)
        parts.append(
            '<model name="hazard_%(t)s_recon_%(i)d">\n'
            '  <static>true</static>\n'
            '  <pose>%(x).3f %(y).3f 0.165 0 0 %(yaw).3f</pose>\n'
            '  <link name="link">\n'
            '    <visual name="marker">\n'
            '      <pose>0 0 0 0 0 0</pose>\n'
            '      <geometry><box><size>0.12 0.12 0.004</size></box></geometry>\n'
            '      <material><diffuse>1 1 1 1</diffuse><pbr><metal>\n'
            '        <albedo_map>model://hazard_marker/materials/textures/%(t)s.png</albedo_map>\n'
            '        <roughness>0.65</roughness><metalness>0</metalness>\n'
            '      </metal></pbr></material>\n'
            '    </visual>\n'
            '  </link>\n'
            '</model>\n' % {'t': marker, 'i': i, 'x': x, 'y': y, 'yaw': yaw})
    return '\n'.join(parts)


def patch_world(world_path, block):
    """替换飞行世界中 drop_1 起、iris 注释止的场景段（其余一字不动）。"""
    with open(world_path, 'r', encoding='utf-8') as f:
        content = f.read()
    anchor = content.index('<name>drop_1')
    start = content.rindex('<include>', 0, anchor)
    end = content.index('<!-- yaw=0')
    with open(world_path, 'w', encoding='utf-8') as f:
        f.write(content[:start] + block + '\n' + content[end:])


def write_truth(truth_path, drop_pts, recon_pts, markers):
    """generated_scene.yaml 同格式（真值感知替身消费 drop_targets 段）。"""
    lines = ['seed: random', 'source: randomize_scene', 'drop_targets:']
    for i, ((x, y), (model, d, s)) in enumerate(zip(drop_pts, DROP_MODELS), 1):
        lines += ['  drop_%d:' % i, '    x: %.3f' % x, '    y: %.3f' % y,
                  '    z: 0.0', '    radius: %.3f' % (d / 2), '    score: %d' % s]
    lines.append('recon_targets:')
    for i, ((x, y), mk) in enumerate(zip(recon_pts, markers), 1):
        lines += ['  recon_%d:' % i, '    x: %.3f' % x, '    y: %.3f' % y,
                  '    z: 0.0', '    marker: %s' % (mk or 'none')]
    with open(truth_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def write_judge_params(path, drop_pts):
    """虚拟判定节点参数（桶真值随布景联动，判分不再吃旧坐标）。"""
    lines = ['/**:', '  ros__parameters:']
    for i, ((x, y), (_m, d, s)) in enumerate(zip(drop_pts, DROP_MODELS), 1):
        lines += ['    drop_%d.x: %.3f' % (i, x), '    drop_%d.y: %.3f' % (i, y),
                  '    drop_%d.radius: %.3f' % (i, d / 2), '    drop_%d.score: %d' % (i, s)]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser(description='随机布景（投放桶/侦察桶/危化品标识）')
    ap.add_argument('--seed', type=int, default=None, help='随机种子（缺省随机）')
    ap.add_argument('--world', default='/home/nvidia/sim_scripts/cuadc_rescue_flight.sdf')
    ap.add_argument('--truth', default='/home/nvidia/cuadc_ws/src/cuadc_rescue_sim/config/generated_scene.yaml')
    ap.add_argument('--judge-params', default='/home/nvidia/sim_scripts/judge_truth_params.yaml')
    ap.add_argument('--no-restart-gz', action='store_true', help='只写文件，不重启 gz')
    args = ap.parse_args()

    seed = args.seed if args.seed is not None else random.randrange(10 ** 6)
    rng = random.Random(seed)

    drop_pts = sample_points(3, DROP_ZONE, DROP_MIN_GAP, rng)
    recon_pts = sample_points(RECON_N, RECON_ZONE, RECON_MIN_GAP, rng)
    markers = [None] * RECON_N
    for mk in rng.sample(MARKERS, MARKER_N):
        while True:
            i = rng.randrange(RECON_N)
            if markers[i] is None:
                markers[i] = mk
                break

    block = build_scene_block(drop_pts, recon_pts, markers, rng)
    patch_world(args.world, block)
    write_truth(args.truth, drop_pts, recon_pts, markers)
    write_judge_params(args.judge_params, drop_pts)

    print('seed=%d' % seed)
    for i, ((x, y), (m, d, _s)) in enumerate(zip(drop_pts, DROP_MODELS), 1):
        print('  投放桶 drop_%d (%s d=%.2f): (%.2f, %.2f)' % (i, m, d, x, y))
    for i, ((x, y), mk) in enumerate(zip(recon_pts, markers), 1):
        print('  侦察桶 recon_%d: (%.2f, %.2f) 标识=%s' % (i, x, y, mk or '无'))
    print('已写: %s\n      %s\n      %s' % (args.world, args.truth, args.judge_params))

    if not args.no_restart_gz:
        import subprocess
        print('重启 gz 服务端载入新场景（SITL 不动，无不稳定窗）...')
        subprocess.run(['pkill', '-f', 'gz [s]im -s'], check=False)
        subprocess.run(['bash', '/home/nvidia/sim_scripts/start_gz.sh'], check=False)
        print('完成。下一步: python3 ~/sim_scripts/fcu_ready.py → fc_sitl_m3.sh / fc_sitl_m2.sh')


if __name__ == '__main__':
    main()
