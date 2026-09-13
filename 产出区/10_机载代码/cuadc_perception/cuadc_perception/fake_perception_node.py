#!/usr/bin/env python3
"""假感知节点：按《接口契约.md》v1.0 发布模拟白筒检测 + 心跳。

用途（M1/M2 阶段不需要真模型就能联调）：
  1. 验证状态机的感知接口链路（M2 接入锁筒逻辑前先跑通数据流）；
  2. 契约校验器的被测对象（check_vision_contract 的标准发布端）；
  3. P0.4 时间同步验收：pipeline_delay_s 设 0.2 注入人工延迟，
     验证 mission 侧 interpolate_odom 的插值补偿（见 时间同步设计.md §5）。

真感知节点（YOLOv8n-seg TensorRT + 单目解算 + 后处理四件）M2 时在本包实现，
必须复用本文件演示的发布语义（stamp=取帧时刻、orientation 字段复用、空帧照发）。
"""

import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header


class FakePerceptionNode(Node):

    def __init__(self):
        super().__init__('fake_perception')

        # ---- 参数 ----
        self.declare_parameter('frame_rate_hz', 30.0)
        self.declare_parameter('pipeline_delay_s', 0.0)   # 取帧→发布的人工延迟（P0.4 测试用）
        self.declare_parameter('noise_std_m', 0.01)       # 模拟检测抖动
        self.declare_parameter('contract_version', 1.0)   # 契约哨兵，见 接口契约.md §1
        self.declare_parameter('frame_id', 'cuadc_body_flu')
        self.declare_parameter('bucket_topic', '/perception/drop_buckets_body')
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        # 模拟三个筒（机体系：x 前 y 左 z 上；筒在飞机下方所以 z 为负）
        self.declare_parameter('bucket_positions_x', [1.5, 1.8, 2.2])
        self.declare_parameter('bucket_positions_y', [0.5, -0.6, 0.1])
        self.declare_parameter('bucket_positions_z', [-1.8, -1.8, -1.8])
        self.declare_parameter('bucket_diameters', [0.25, 0.20, 0.15])
        self.declare_parameter('bucket_confidences', [0.90, 0.85, 0.80])

        gp = self.get_parameter
        self.rate_hz = max(1.0, gp('frame_rate_hz').value)
        self.delay_s = max(0.0, gp('pipeline_delay_s').value)
        self.noise = max(0.0, gp('noise_std_m').value)
        self.version = gp('contract_version').value
        self.frame_id = gp('frame_id').value
        self.bx = list(gp('bucket_positions_x').value)
        self.by = list(gp('bucket_positions_y').value)
        self.bz = list(gp('bucket_positions_z').value)
        self.bd = list(gp('bucket_diameters').value)
        self.bc = list(gp('bucket_confidences').value)
        if not (len(self.bx) == len(self.by) == len(self.bz) ==
                len(self.bd) == len(self.bc)):
            raise RuntimeError('bucket_* 列表参数长度必须一致')

        qos = qos_profile_sensor_data
        self.bucket_pub = self.create_publisher(
            PoseArray, gp('bucket_topic').value, qos)
        self.heartbeat_pub = self.create_publisher(
            Header, gp('heartbeat_topic').value, qos)
        self.frame_count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f'假感知启动: {self.rate_hz:.0f}Hz, {len(self.bx)} 个模拟筒, '
            f'注入延迟={self.delay_s:.2f}s, 契约版本={self.version}')

    def on_timer(self):
        # 关键契约：stamp = 取帧时刻（这里 = now − 注入延迟），不是发布时刻
        capture_time = self.get_clock().now() - Duration(seconds=self.delay_s)

        msg = PoseArray()
        msg.header.stamp = capture_time.to_msg()
        msg.header.frame_id = self.frame_id
        for x, y, z, d, c in zip(self.bx, self.by, self.bz, self.bd, self.bc):
            pose = Pose()
            pose.position.x = x + random.gauss(0.0, self.noise)
            pose.position.y = y + random.gauss(0.0, self.noise)
            pose.position.z = z + random.gauss(0.0, self.noise)
            pose.orientation.x = d      # 复用：筒口直径（m）
            pose.orientation.y = c      # 复用：置信度 [0,1]
            pose.orientation.z = self.version  # 复用：契约版本哨兵
            pose.orientation.w = 0.0    # 保留
            msg.poses.append(pose)
        self.bucket_pub.publish(msg)

        # 心跳与检测同源同节奏，空帧也发（fail-closed：停发 = 故障）
        hb = Header()
        hb.stamp = capture_time.to_msg()
        hb.frame_id = 'fake_perception'
        self.heartbeat_pub.publish(hb)

        self.frame_count += 1
        if self.frame_count % self.rate_hz == 0:  # 每秒一条，防刷屏
            self.get_logger().debug(f'published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = FakePerceptionNode()
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
