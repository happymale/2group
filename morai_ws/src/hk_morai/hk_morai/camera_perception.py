"""HEVEN 2026 MORAI 카메라 인지 (camera_perception). 참가자가 채울 곳.

카메라 4대의 압축 영상(sensor_msgs/CompressedImage, JPEG, 20 Hz)을 받아 self.images[이름]에 둡니다.
  front          /hk/sensors/camera/front/compressed           1280×720, 화각 90°
  left           /hk/sensors/camera/left/compressed            640×480, 화각 130°, 왼쪽 70°
  right          /hk/sensors/camera/right/compressed           640×480, 화각 130°, 오른쪽 70°
  traffic_light  /hk/sensors/camera/traffic_light/compressed
(해상도·위치는 config/sensor_mounts.yaml에 있습니다.) 지금은 받기만 하고 아무것도 내지 않습니다.
기본 주행(bringup)에서는 띄우지 않고, perception:=true일 때만 뜹니다. 영상을 풀면 CPU를 많이 쓰니 필요한
카메라만 푸세요.

쓰는 방법 예 (on_image 안에서):
    import cv2, numpy as np
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)   # BGR 이미지
"""
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage

CAMERAS = ('front', 'left', 'right', 'traffic_light')


class CameraPerception(Node):
    def __init__(self):
        super().__init__('hk_morai_camera_perception')
        self.images = {name: None for name in CAMERAS}
        for name in CAMERAS:
            self.create_subscription(CompressedImage, f'/hk/sensors/camera/{name}/compressed',
                                     partial(self.on_image, name), qos_profile_sensor_data)

    def on_image(self, name, m):
        self.images[name] = m            # 여기서 영상을 처리하세요


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
