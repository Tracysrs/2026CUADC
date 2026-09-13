#!/usr/bin/env python3
"""临时探针：对比 odom 与感知帧的时间戳基准（P0.4 拒帧定位用，用完可删）。"""
import time

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Probe(Node):

    def __init__(self):
        super().__init__('stamp_probe')
        self.odom = None
        self.pose = None
        self.pose_n = 0
        self.visible = -1
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseArray, '/perception/drop_buckets_body', self.on_pose,
            qos_profile_sensor_data)

    def on_odom(self, m):
        self.odom = m.header.stamp

    def on_pose(self, m):
        self.pose = m.header.stamp
        self.pose_n += 1
        self.visible = len(m.poses)


def s2f(stamp):
    return None if stamp is None else stamp.sec + stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = Probe()
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - last_print >= 2.0:
            last_print = time.time()
            o, p = s2f(node.odom), s2f(node.pose)
            d = None if (o is None or p is None) else p - o
            print('t=%.1f odom=%s pose=%s delta=%.3f poses_n=%d visible=%d'
                  % (time.time() - t0, o, p,
                     d if d is not None else float('nan'),
                     node.pose_n, node.visible), flush=True)
    o, p = s2f(node.odom), s2f(node.pose)
    print('FINAL odom=%s pose=%s pose_msgs=%d' % (o, p, node.pose_n))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
