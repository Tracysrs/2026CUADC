#!/usr/bin/env python3
"""P0.4 时间同步验收节点（时间同步设计.md §5，一条命令完成验收）：

    ros2 run cuadc_perception p04_odom_perception_node --ros-args \
        -p delay_s:=0.2 -p duration_s:=10.0

不需要 MAVROS / SITL / Gazebo——验收对象是 mission_node 的真实插值管线
（odom_callback → odom_history_ → interpolate_odom → body_to_local_at），
本节点同时扮演三个角色：

  1. 合成里程计：50Hz 发布 /mavros/local_position/odom，飞机沿世界 +x 以
     speed_m_s 匀速直线（可加高斯噪声）；
  2. 世界静止目标感知：世界系 (target_x, target_y, 0) 一个白筒。取帧时刻
     stamp = now − delay_s，用【取帧瞬间】的飞机位姿把目标换算成机体系 FLU
     发布（契约 §1：frame_id=cuadc_body_flu、orientation.z=1.0 哨兵、空帧照发）；
  3. 验收判读：订阅 /cuadc/debug/buckets_world（mission 的插值换算输出），
     跳过 warmup_s 后统计世界坐标波动。

通过标准：波动 std < 0.05m。不插值的对照偏差 ≈ speed × delay
（2m/s × 0.2s = 40cm）。反向用例：delay_s > perception_max_delay_s(1.5) →
mission 必须全部拒收（本节点收不到任何调试输出 → PASS；mission 日志出现
"感知帧被拒"节流告警）。退出码 0=PASS / 1=FAIL（可接 CI）。
"""

import random
import statistics
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class AcceptanceDone(Exception):
    """验收完成：携带退出码，从回调里干净地结束 spin（禁止回调内 shutdown——会死锁）。"""

    def __init__(self, code: int):
        super().__init__(f'exit={code}')
        self.code = code


class P04AcceptanceNode(Node):

    def __init__(self):
        super().__init__('p04_odom_perception')

        # ---- 场景参数 ----
        self.declare_parameter('speed_m_s', 2.0)        # SSOT §5.4 搜索速度
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('perception_rate_hz', 20.0)
        self.declare_parameter('delay_s', 0.2)          # 取帧→发布的管线延迟
        self.declare_parameter('duration_s', 10.0)      # 验收窗口
        self.declare_parameter('warmup_s', 3.0)         # 跳过启动段
        self.declare_parameter('target_x', 10.0)        # 目标世界系坐标（静止）
        self.declare_parameter('target_y', 1.0)
        self.declare_parameter('target_diameter', 0.20)
        self.declare_parameter('odom_noise_std', 0.005) # odom 量测噪声
        self.declare_parameter('tolerance_m', 0.05)     # 验收线（SSOT P0.4）

        gp = self.get_parameter
        self.speed = gp('speed_m_s').value
        self.delay = gp('delay_s').value
        self.duration = gp('duration_s').value
        self.warmup = gp('warmup_s').value
        self.tx = gp('target_x').value
        self.ty = gp('target_y').value
        self.diameter = gp('target_diameter').value
        self.noise = gp('odom_noise_std').value
        self.tolerance = gp('tolerance_m').value
        self.expect_reject = self.delay > 1.5           # 与 mission perception_max_delay_s 对齐

        self._rng = random.Random(2026)

        self.odom_pub = self.create_publisher(
            Odometry, '/mavros/local_position/odom', qos_profile_sensor_data)
        self.det_pub = self.create_publisher(
            PoseArray, '/perception/drop_buckets_body', qos_profile_sensor_data)
        self.world_sub = self.create_subscription(
            PoseArray, '/cuadc/debug/buckets_world', self.on_world, 10)

        self.t0 = self.get_clock().now()
        self.samples = []                # (x, y) 插值换算结果
        self.n_reject_warmup = 0
        self.odom_timer = self.create_timer(1.0 / gp('odom_rate_hz').value, self.on_odom)
        self.det_timer = self.create_timer(
            1.0 / gp('perception_rate_hz').value, self.on_perception)
        self.create_timer(self.duration, self.finish)

        self.get_logger().info(
            f'P0.4 验收启动: 速度={self.speed}m/s 延迟={self.delay}s '
            f'目标世界系=({self.tx},{self.ty}) 窗口={self.duration}s '
            f'模式={"应当全拒" if self.expect_reject else "应当插值通过"}')

    # ---- 角色 1：合成匀速直线里程计（50Hz）----
    def on_odom(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.speed * t + self._rng.gauss(0, self.noise)
        msg.pose.pose.position.y = self._rng.gauss(0, self.noise)
        msg.pose.pose.position.z = 2.0
        msg.pose.pose.orientation.w = 1.0            # yaw=0（机头朝 +x）
        msg.twist.twist.linear.x = self.speed
        self.odom_pub.publish(msg)

    # ---- 角色 2：世界静止目标 → 取帧瞬间机体系（20Hz，带延迟）----
    def on_perception(self):
        now = self.get_clock().now()
        t_cap = (now - self.t0).nanoseconds * 1e-9 - self.delay
        if t_cap < 0.0:
            self.n_reject_warmup += 1
            return                                    # 取帧时刻早于起飞，跳过
        px = self.speed * t_cap                       # 取帧瞬间飞机位置
        msg = PoseArray()
        msg.header.stamp = (self.t0 + rclpy.duration.Duration(seconds=t_cap)).to_msg()
        msg.header.frame_id = 'cuadc_body_flu'
        pose = Pose()
        pose.position.x = self.tx - px                # yaw=0：世界差 = 机体系
        pose.position.y = self.ty
        pose.position.z = -2.0                        # 筒在飞机下方
        pose.orientation.x = self.diameter            # 契约：直径
        pose.orientation.y = 0.90                     # 契约：置信度
        pose.orientation.z = 1.0                      # 契约：版本哨兵
        pose.orientation.w = 0.0
        msg.poses.append(pose)
        self.det_pub.publish(msg)

    # ---- 角色 3：判读 ----
    def on_world(self, msg: PoseArray):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if elapsed < self.warmup:
            return
        for p in msg.poses:
            self.samples.append((p.position.x, p.position.y))

    def finish(self):
        out = []
        out.append('=' * 56)
        if self.expect_reject:
            if self.samples:
                out.append(f'FAIL 反向用例：delay={self.delay}s>1.5s 仍有 {len(self.samples)} '
                           f'个换算输出——超龄帧未被拒收！')
                code = 1
            else:
                out.append(f'PASS 反向用例：delay={self.delay}s>1.5s，超龄帧全部被拒收'
                           f'（启动期跳过 {self.n_reject_warmup} 帧未发布）')
                code = 0
        else:
            n = len(self.samples)
            if n < 20:
                out.append(f'FAIL 样本不足（{n} < 20）——检查 mission /cuadc/debug/buckets_world')
                code = 1
            else:
                xs = [s[0] for s in self.samples]
                ys = [s[1] for s in self.samples]
                sx, sy = statistics.pstdev(xs), statistics.pstdev(ys)
                mx, my = statistics.mean(xs), statistics.mean(ys)
                bias = max(abs(mx - self.tx), abs(my - self.ty))
                out.append(f'样本 {n} 个 | 目标 ({self.tx:.2f}, {self.ty:.2f}) | '
                           f'均值 ({mx:.3f}, {my:.3f}) | 偏置 {bias*100:.1f}cm')
                out.append(f'波动 std = ({sx*100:.2f}, {sy*100:.2f}) cm，'
                           f'验收线 {self.tolerance*100:.0f}cm | '
                           f'不插值对照 ≈ {self.speed * self.delay * 100:.0f}cm')
                if sx < self.tolerance and sy < self.tolerance:
                    out.append('PASS P0.4：插值补偿有效，波动 < 验收线')
                    code = 0
                else:
                    out.append('FAIL P0.4：波动超线')
                    code = 1
        out.append('=' * 56)
        print('\n'.join(out), flush=True)
        raise AcceptanceDone(code)


def main(args=None):
    rclpy.init(args=args)
    node = P04AcceptanceNode()
    code = 1
    try:
        rclpy.spin(node)
    except AcceptanceDone as done:
        code = done.code
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
