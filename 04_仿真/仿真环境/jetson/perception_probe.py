#!/usr/bin/env python3
"""诊断探针：录 /perception/drop_buckets_body 全量内容（检测质量分析用）。"""
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('perception_probe')
    lines = []

    def cb(msg):
        poses = [(round(p.position.x, 3), round(p.position.y, 3),
                  round(p.orientation.x, 3), round(p.orientation.y, 2)) for p in msg.poses]
        lines.append('t=%.1f n=%d %s' % (time.time(), len(poses), poses))
        if len(lines) % 25 == 0:
            with open('/tmp/perception_probe.log', 'w') as f:
                f.write('\n'.join(lines))
                f.flush()

    node.create_subscription(
        PoseArray, '/perception/drop_buckets_body', cb, qos_profile_sensor_data)
    end = time.time() + 300
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    with open('/tmp/perception_probe.log', 'w') as f:
        f.write('\n'.join(lines))


if __name__ == '__main__':
    main()
