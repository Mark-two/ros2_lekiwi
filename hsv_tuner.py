#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def nothing(x):
    pass

class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()
        # 订阅原始彩色图像
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.callback, 10)

        # 创建一个带滑动条的操作面板
        cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('HSV Tuner', 400, 300)
        
        # 针对红色的两个色相(H)区间 (0~X 和 Y~179)
        cv2.createTrackbar('H_min1', 'HSV Tuner', 0, 179, nothing)
        cv2.createTrackbar('H_max1', 'HSV Tuner', 5, 179, nothing)   # 💥 关键：把上限压到 5 以下，避开皮肤的 10~20
        cv2.createTrackbar('H_min2', 'HSV Tuner', 170, 179, nothing)
        cv2.createTrackbar('H_max2', 'HSV Tuner', 179, 179, nothing)

        # 饱和度(S) 和 明度(V)
        cv2.createTrackbar('S_min', 'HSV Tuner', 70, 255, nothing)
        cv2.createTrackbar('S_max', 'HSV Tuner', 255, 255, nothing)
        cv2.createTrackbar('V_min', 'HSV Tuner', 50, 255, nothing)
        cv2.createTrackbar('V_max', 'HSV Tuner', 255, 255, nothing)

        self.get_logger().info("🎛️ HSV 调参器已启动！请调节滑块，直到右侧的 Mask 图像中，只有红点是白色的！")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 实时读取滑块的值
        h1_min = cv2.getTrackbarPos('H_min1', 'HSV Tuner')
        h1_max = cv2.getTrackbarPos('H_max1', 'HSV Tuner')
        h2_min = cv2.getTrackbarPos('H_min2', 'HSV Tuner')
        h2_max = cv2.getTrackbarPos('H_max2', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Tuner')

        # 组合 Mask
        mask1 = cv2.inRange(hsv, np.array([h1_min, s_min, v_min]), np.array([h1_max, s_max, v_max]))
        mask2 = cv2.inRange(hsv, np.array([h2_min, s_min, v_min]), np.array([h2_max, s_max, v_max]))
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 显示原图和黑白掩膜 (Mask)
        # 你的目标是：让 Mask 图像里，你的大拇指全是黑的，手机红点全是白的！
        cv2.imshow('Original', cv2.resize(frame, (640, 480)))
        cv2.imshow('Mask (White is Target)', cv2.resize(mask, (640, 480)))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()