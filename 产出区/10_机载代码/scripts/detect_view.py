#!/usr/bin/env python3
"""带识别框的实时查看器：/camera/image_raw/compressed → YOLO engine → 画框显示。

用法（SSH 里启动，窗口显示在 Jetson 机身屏幕）：
    source /opt/ros/humble/setup.bash
    DISPLAY=:0 nohup python3 ~/detect_view.py > /tmp/detect_view.log 2>&1 < /dev/null &
按 q 退出。

框色语义（对应 §6 拒识双阈值）：
- 绿框：conf ≥ 0.7 且 margin ≥ 0.3（能过帧级门控，进入融合）
- 橙框：conf ≥ 0.7 但 margin < 0.3（两类竞争，灰区，会触发原图重推）
- 红框：conf < 0.7（帧级门控直接丢弃，不进融合）
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

ENGINE = '/home/nvidia/cuadc_models/best.engine'
CONF_GATE = 0.7    # 帧级拒识 top1 阈值（§6，与 hazard_recon_params.yaml 一致）
MARGIN_GATE = 0.3  # 帧级拒识 top1-top2 阈值
IMGSZ = 640

# cv2.putText 不支持中文（显示为 ????），engine 内嵌类别名是中文 → 查表转拼音
# 顺序 = 附件 11 十类（§2.1，id 0..9）
PINYIN = {
    '爆炸品': 'baozhapin',
    '不燃气体': 'buran-qiti',
    '刺激性': 'cijixing',
    '放射性物品': 'fangshexing',
    '腐蚀品': 'fushipin',
    '生物危害': 'shengwu-weihai',
    '遇湿易燃物品': 'yushi-yiran',
    '有毒品': 'youdupin',
    '自燃物品': 'ziran-wupin',
    '易燃': 'yiran',
}


class DetectView(Node):

    def __init__(self):
        super().__init__('detect_view')
        self.model = YOLO(ENGINE, task='detect')
        # 黑帧预热 3 帧（§4.3 部署铁律：首帧秒级初始化）
        self.model.predict(np.zeros((640, 640, 3), dtype=np.uint8),
                           imgsz=IMGSZ, verbose=False)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image,
            qos_profile_sensor_data)
        self.n, self.t0, self.fps, self.ms = 0, time.time(), 0.0, 0.0

    def on_image(self, msg):
        t0 = time.time()
        buf = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        r = self.model.predict(frame, imgsz=IMGSZ, conf=0.10, verbose=False)[0]
        for b in r.boxes:
            cid = int(b.cls)
            cls = r.names[cid]
            label = f'{cid}-{PINYIN.get(cls, cls)}'
            conf = float(b.conf)
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            # 排序取 top2 求 margin（与感知端同口径；仅 1 框时 margin=conf）
            confs = sorted((float(x.conf) for x in r.boxes), reverse=True)
            margin = conf - confs[1] if len(confs) > 1 else conf
            if conf < CONF_GATE:
                color = (0, 0, 255)      # 红：过不了门控
            elif margin < MARGIN_GATE:
                color = (0, 165, 255)    # 橙：灰区竞争
            else:
                color = (0, 255, 0)      # 绿：进融合
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, max(y1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        self.ms = 1000 * (time.time() - t0)
        self.n += 1
        now = time.time()
        if now - self.t0 >= 1.0:
            self.fps = self.n / (now - self.t0)
            self.n, self.t0 = 0, now
        cv2.putText(frame,
                    f'{self.fps:.1f} FPS  infer {self.ms:.0f}ms  '
                    f'green>=gate  (q quit)',
                    (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow('cuadc detect view', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = DetectView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass


if __name__ == '__main__':
    main()
