#!/usr/bin/env python3
"""契约校验器：对 /perception/drop_buckets_body + /perception/heartbeat 做在线体检。

用法（被测对象可以是假感知节点，也可以是真感知节点——上线门槛工具）：
    ros2 run cuadc_perception check_vision_contract --ros-args \
        -p duration_s:=10.0 -p min_rate_hz:=15.0
逐项打印 PASS/FAIL，全部通过退出码 0，任一失败退出码 1（可接 CI/上机前检查）。
校验项与《接口契约.md》§5 一一对应；契约升级时必须同步改本脚本。
"""

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray


class ContractChecker(Node):

    def __init__(self):
        super().__init__('contract_checker')

        self.declare_parameter('duration_s', 10.0)        # 采样窗口
        self.declare_parameter('min_rate_hz', 15.0)       # 频率下限（验收 25，联调放宽）
        self.declare_parameter('max_delay_s', 1.5)        # 帧龄上界（契约 §3）
        self.declare_parameter('future_tol_s', 0.05)      # 未来戳容差
        self.declare_parameter('heartbeat_timeout_s', 1.5)
        self.declare_parameter('expected_frame_id', 'cuadc_body_flu')
        self.declare_parameter('expected_version', 1.0)
        self.declare_parameter('max_poses_per_frame', 8)
        self.declare_parameter('diameter_min_m', 0.05)
        self.declare_parameter('diameter_max_m', 0.50)
        self.declare_parameter('dedup_warn_m', 0.02)      # 同帧 <2cm 视为疑似未去重

        gp = self.get_parameter
        self.duration = float(gp('duration_s').value)
        self.min_rate = float(gp('min_rate_hz').value)
        self.max_delay = float(gp('max_delay_s').value)
        self.future_tol = float(gp('future_tol_s').value)
        self.hb_timeout = float(gp('heartbeat_timeout_s').value)
        self.expected_frame_id = gp('expected_frame_id').value
        self.expected_version = float(gp('expected_version').value)
        self.max_poses = int(gp('max_poses_per_frame').value)
        self.d_min = float(gp('diameter_min_m').value)
        self.d_max = float(gp('diameter_max_m').value)
        self.dedup_warn = float(gp('dedup_warn_m').value)

        self.det_count = 0
        self.hb_count = 0
        self.hb_last_recv = None
        self.det_fail_examples = []   # (原因, 最多存 5 条)
        self.det_warn_examples = []
        self.stamp_violation = False  # 延迟/未来戳违约（可能是 use_sim_time 没对齐）

        qos = qos_profile_sensor_data
        self.create_subscription(
            PoseArray, '/perception/drop_buckets_body', self.on_buckets, qos)
        self.create_subscription(Header, '/perception/heartbeat', self.on_hb, qos)

        self._finished = False
        self.create_timer(self.duration, self._finish)
        self.get_logger().info(
            f'契约校验启动：采样 {self.duration:.0f}s，频率下限 {self.min_rate}Hz …')

    # ------------------------------------------------------------------ 回调
    def on_hb(self, msg: Header):
        self.hb_count += 1
        now = self.get_clock().now()
        if self.hb_last_recv is not None and (now - self.hb_last_recv).nanoseconds > \
                int(self.hb_timeout * 1e9):
            self._add_fail('心跳间隔超时 (>1.5s)')
        self.hb_last_recv = now

    def on_buckets(self, msg: PoseArray):
        self.det_count += 1
        now = self.get_clock().now()
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        except Exception:  # noqa: BLE001
            self._add_fail('header.stamp 无法解析')
            return

        age = (now - stamp).nanoseconds / 1e9
        future = (stamp - now).nanoseconds / 1e9
        if future > self.future_tol:
            self._add_fail(f'未来戳超容差 {future * 1000:.0f}ms')
            self.stamp_violation = True
        elif age > self.max_delay:
            self._add_fail(f'帧龄 {age:.2f}s 超上界 {self.max_delay}s')
            self.stamp_violation = True

        if msg.header.frame_id != self.expected_frame_id:
            self._add_fail(f'frame_id={msg.header.frame_id} ≠ {self.expected_frame_id}')
        if len(msg.poses) > self.max_poses:
            self._add_fail(f'单帧 pose 数 {len(msg.poses)} > 上限 {self.max_poses}')

        versions = set()
        for i, pose in enumerate(msg.poses):
            p = pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                self._add_fail(f'pose[{i}] 坐标含 NaN/Inf')
            if not 0.0 <= pose.orientation.y <= 1.0:
                self._add_fail(f'pose[{i}] 置信度 {pose.orientation.y:.2f} 越界 [0,1]')
            if not self.d_min <= pose.orientation.x <= self.d_max:
                self._add_fail(f'pose[{i}] 直径 {pose.orientation.x:.3f}m 越界')
            versions.add(round(float(pose.orientation.z), 3))
            if pose.orientation.w != 0.0:
                self._add_fail(f'pose[{i}] orientation.w 应保留 0.0')
        if len(versions) > 1:
            self._add_fail(f'契约版本哨兵不一致: {versions}')
        elif versions and self.expected_version not in versions:
            self._add_fail(
                f'契约版本 {versions} ≠ 期望 {self.expected_version}（契约升级未同步？）')

        # 同帧去重是感知端的责任，这里只告警（SSOT §4.1 中心距 <2cm 必须合并）
        for i in range(len(msg.poses)):
            for j in range(i + 1, len(msg.poses)):
                a, b = msg.poses[i].position, msg.poses[j].position
                dist = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                if dist < self.dedup_warn:
                    self._add_warn(f'pose[{i}]/[{j}] 距 {dist * 100:.1f}cm 疑似未去重')

    def _add_fail(self, reason):
        self.get_logger().warning(f'FAIL: {reason}')
        if len(self.det_fail_examples) < 5:
            self.det_fail_examples.append(reason)

    def _add_warn(self, reason):
        self.get_logger().warning(f'WARN: {reason}')
        if len(self.det_warn_examples) < 5:
            self.det_warn_examples.append(reason)

    # ------------------------------------------------------------------ 收尾
    def _finish(self):
        self._finished = True
        rclpy.shutdown()  # 让 spin() 返回

    def report(self):
        rate = self.det_count / max(0.1, self.duration)
        hb_rate = self.hb_count / max(0.1, self.duration)
        checks = [
            (f'检测频率 {rate:.1f}Hz ≥ 下限 {self.min_rate}Hz', rate >= self.min_rate),
            (f'心跳频率 {hb_rate:.1f}Hz 且无超时间隔', self.hb_count > 0),
            ('消息 stamp 合法（延迟/未来戳容差内）', not self.stamp_violation),
            ('字段语义全部合规（哨兵/置信度/直径/有限性/单帧上限）',
             len(self.det_fail_examples) == 0),
        ]
        print('\n========== 契约校验报告 ==========')
        print(f'采样窗口 {self.duration:.0f}s：检测帧 {self.det_count}，心跳 {self.hb_count}')
        all_ok = True
        for name, ok in checks:
            print(f'  [{"PASS" if ok else "FAIL"}] {name}')
            all_ok = all_ok and ok
        if self.det_fail_examples:
            print('  失败样例（最多 5 条）:')
            for r in self.det_fail_examples:
                print(f'    - {r}')
        if self.det_warn_examples:
            print('  告警（不阻断）:')
            for r in self.det_warn_examples:
                print(f'    - {r}')
        print('==================================')
        print('结论: ' + ('契约合规 ✓' if all_ok else '契约违约 ✗（感知端修复前不许上机）'))
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = ContractChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        all_ok = node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
