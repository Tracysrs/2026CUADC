#!/usr/bin/env python3
"""场景真值感知替身：把 generated_scene.yaml 的投放桶真值按相机可见性发布为
《接口契约.md》v1.0 机体系 PoseArray，驱动 mission 状态机在 SITL 里跑真
SEARCH 锁定 → ALIGN → RELEASE 全链路（M3 仿真判分闭环，配合 sim_release_bridge
+ virtual_drop_judge_node 实现"投放 2/2"验收）。

与 fake_perception_node.py 的区别：那个发布固定机体系坐标（联调数据流用），
本节点订阅 odom 做世界系→机体系反变换 + 下视相机可见性门控——飞机飞到哪、
筒就该在哪被"看见"，SEARCH 蛇形扫描逐步建筒位地图的语义是真实的。

M2 真感知（YOLOv8n-seg + 单目解算）落地后本节点退役，但发布语义
（stamp=取帧时刻、orientation 字段复用、空帧照发、可见性门控）保持一致。
"""

import math
import os
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Header
import yaml

# 下视相机（iris_d435i_airframe/model.sdf）：hfov 张在 848px 宽方向（沿机头），
# 横向沿 480px 方向。可见性半宽 = 高度 × tan(半角)。
_TAN_HALF_HFOV = math.tan(1.5 / 2.0)          # ≈0.932，沿飞行方向
_TAN_HALF_VFOV = _TAN_HALF_HFOV * 480.0 / 848.0  # ≈0.527，横向


class SceneTruthPerceptionNode(Node):

    def __init__(self):
        super().__init__('scene_truth_perception')

        # ---- 参数 ----
        self.declare_parameter(
            'scene_yaml',
            os.path.expanduser(
                '~/cuadc_ws/src/cuadc_rescue_sim/config/generated_scene.yaml'))
        self.declare_parameter('frame_rate_hz', 25.0)      # 契约 ≥25Hz
        self.declare_parameter('pipeline_delay_s', 0.1)    # 取帧→发布人工延迟
        self.declare_parameter('noise_std_m', 0.01)        # 检测抖动（陀螺/解算）
        self.declare_parameter('confidence', 0.90)         # 置信度（>0.25 契约下限）
        self.declare_parameter('contract_version', 1.0)    # 契约哨兵
        self.declare_parameter('frame_id', 'cuadc_body_flu')
        self.declare_parameter('bucket_topic', '/perception/drop_buckets_body')
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        self.declare_parameter('bucket_top_z', 0.30)       # 投放桶高 30cm（RULE_MAPPING）
        self.declare_parameter('min_visible_alt_m', 0.3)   # 相机离筒口低于此值不报
        self.declare_parameter('footprint_margin', 1.1)    # 可见性余量（边缘畸变）

        gp = self.get_parameter
        self.scene_path = gp('scene_yaml').value
        self.rate_hz = max(1.0, gp('frame_rate_hz').value)
        self.delay_s = max(0.0, gp('pipeline_delay_s').value)
        self.noise = max(0.0, gp('noise_std_m').value)
        self.confidence = gp('confidence').value
        self.version = gp('contract_version').value
        self.frame_id = gp('frame_id').value
        self.bucket_top_z = gp('bucket_top_z').value
        self.min_alt = gp('min_visible_alt_m').value
        self.margin = gp('footprint_margin').value

        self.buckets = self._load_scene(self.scene_path)
        if not self.buckets:
            raise RuntimeError(f'{self.scene_path} 中没有 drop_targets 真值')

        qos = qos_profile_sensor_data
        self.bucket_pub = self.create_publisher(
            PoseArray, gp('bucket_topic').value, qos)
        self.heartbeat_pub = self.create_publisher(
            Header, gp('heartbeat_topic').value, qos)
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.on_odom, qos)

        self.have_odom = False
        self.ox = self.oy = self.oz = 0.0
        self.yaw = 0.0
        # 时间基准 = 最新 odom 的 header.stamp（与 P0.4 插值严格同源——
        # mavros odom 用 FCU 时间基准，与节点墙钟不同源，用墙钟必被拒帧）
        self.last_odom_stamp = None
        self.frame_count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        names = ', '.join(f'{b["id"]}(d={b["diameter"]:.2f})' for b in self.buckets)
        self.get_logger().info(
            f'场景真值感知启动: {self.rate_hz:.0f}Hz, {len(self.buckets)} 筒 [{names}], '
            f'场景={self.scene_path}, 延迟={self.delay_s:.2f}s')

    @staticmethod
    def _load_scene(path):
        """读 generated_scene.yaml 的 drop_targets → [{id,x,y,diameter}]。"""
        if not os.path.isfile(path):
            raise RuntimeError(f'场景真值文件不存在: {path}（先同步 cuadc_rescue_sim）')
        with open(path, 'r', encoding='utf-8') as f:
            scene = yaml.safe_load(f)
        buckets = []
        for bid, t in (scene.get('drop_targets') or {}).items():
            buckets.append({
                'id': bid,
                'x': float(t['x']),
                'y': float(t['y']),
                'diameter': 2.0 * float(t['radius']),   # 真值给半径，契约要直径
            })
        return buckets

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ox, self.oy, self.oz = p.x, p.y, p.z
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom_stamp = msg.header.stamp
        self.have_odom = True

    def _visible_body_positions(self):
        """世界系筒 → 机体系（x前 y左 z上），只保留下视相机视野内的。"""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        h = max(self.min_alt, self.oz - self.bucket_top_z)   # 相机离筒口高度
        along_half = h * _TAN_HALF_HFOV * self.margin
        across_half = h * _TAN_HALF_VFOV * self.margin
        out = []
        for b in self.buckets:
            dx, dy = b['x'] - self.ox, b['y'] - self.oy
            bx = c * dx + s * dy          # R(-yaw)：世界系 → 机体系
            by = -s * dx + c * dy
            if abs(bx) <= along_half and abs(by) <= across_half:
                out.append((b, bx, by, self.bucket_top_z - self.oz))  # z 负=筒在下方
        return out

    def on_timer(self):
        # 契约：stamp = 取帧时刻（= 最新 odom 戳 − 注入延迟），不是发布时刻；
        # 与 odom 同源才能过任务侧 P0.4 插值（墙钟戳会被整帧拒用）
        if not self.have_odom or self.last_odom_stamp is None:
            return   # 无 odom 无时间基准，首帧前不发（任务侧也还没法插值）
        try:
            capture_time = Time.from_msg(self.last_odom_stamp) - \
                Duration(seconds=self.delay_s)

            msg = PoseArray()
            msg.header.stamp = capture_time.to_msg()
            msg.header.frame_id = self.frame_id
            for b, bx, by, bz in self._visible_body_positions():
                pose = Pose()
                pose.position.x = bx + random.gauss(0.0, self.noise)
                pose.position.y = by + random.gauss(0.0, self.noise)
                pose.position.z = bz
                pose.orientation.x = b['diameter']    # 复用：筒口直径（m）
                pose.orientation.y = self.confidence  # 复用：置信度 [0,1]
                pose.orientation.z = self.version     # 复用：契约版本哨兵
                pose.orientation.w = 0.0              # 保留
                msg.poses.append(pose)
            self.bucket_pub.publish(msg)

            # 心跳与检测同源同节奏，空帧也发（fail-closed：停发 = 故障）
            hb = Header()
            hb.stamp = capture_time.to_msg()
            hb.frame_id = 'scene_truth_perception'
            self.heartbeat_pub.publish(hb)

            self.frame_count += 1
            if self.frame_count % int(self.rate_hz) == 0:  # 每秒一条，防刷屏
                self.get_logger().debug(
                    f'published {self.frame_count} frames, '
                    f'visible={len(msg.poses)} at ({self.ox:.1f},{self.oy:.1f})')
        except Exception as e:   # 单拍异常不杀 spin（'context is invalid' 实测会崩节点）
            self.get_logger().error(f'on_timer 异常: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SceneTruthPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
