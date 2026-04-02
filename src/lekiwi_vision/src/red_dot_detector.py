#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedDotDetectorNode(Node):
    def __init__(self):
        super().__init__('red_dot_detector')
        self.bridge = CvBridge()
        
        # 存放相机内参
        self.fx = self.fy = self.cx = self.cy = None
        
        # 1. 订阅相机内参 (只拿一次就够了)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
            
        # 2. 建立时间同步订阅器：将彩色图和对齐后的深度图“绑”在一起
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
            
        # 允许 0.1 秒的时间差，把两张图打包送给处理函数
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # 3. 创建发布器：发布 3D 坐标数组和 Debug 图像
        self.pose_pub = self.create_publisher(PoseArray, '/lekiwi/vision/red_dots_3d', 10)
        self.debug_pub = self.create_publisher(Image, '/lekiwi/vision/debug_image', 10)

        self.get_logger().info("🚀 极简流红点 3D 探测雷达已启动！等待视频流...")

    def info_callback(self, msg):
        """ 提取相机内参 (针孔相机模型) """
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.get_logger().info(f"✅ 成功获取相机内参: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
            # 拿到内参后就可以销毁这个订阅器了，节省资源
            self.destroy_subscription(self.info_sub)

    def sync_callback(self, color_msg, depth_msg):
        """ 核心：处理同步到达的彩色图和深度图 """
        if self.fx is None:
            return # 内参还没拿到，先不处理

        try:
            # ROS 图像转 OpenCV 格式
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            # 深度图通常是 16UC1 (16位无符号整型，单位是毫米)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # ==========================================
        # 1. 寻找红点的 2D 像素坐标 (纯净无干涉版)
        # ==========================================
        # 💥 删掉高斯模糊，原汁原味转换
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 黄金 HSV 阈值 (完全信任你调出的参数)
        lower_red1 = np.array([0, 213, 50])
        upper_red1 = np.array([8, 255, 255]) 
        lower_red2 = np.array([170, 213, 50])  # H2下限给点余量到 170
        upper_red2 = np.array([179, 255, 255]) # OpenCV 里 H 最大是 179
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 💥 删掉致死的 MORPH_OPEN，我们只做一次轻微的“膨胀”，让红点更饱满一点
        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.dilate(red_mask, kernel, iterations=1)

        # 寻找轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 💥 放宽物理限制：只要不是 10 个像素以下的灰尘，全放进来！
            if 10 < area < 2000: 
                perimeter = cv2.arcLength(cnt, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * (area / (perimeter * perimeter))
                    # 💥 放宽圆度：0.3 以上的椭圆/多边形全认，只要别是一条线就行
                    if circularity > 0.3: 
                        valid_contours.append(cnt)

        # 按面积排序，只取前 2 个最大的红点
        valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)[:2]

        pose_array_msg = PoseArray()
        pose_array_msg.header = color_msg.header

        # ==========================================
        # 2. 提取 3D 坐标并绘制 Debug 图像
        # ==========================================
        for i, cnt in enumerate(valid_contours):
            # ...(剩下的计算 M, x, y, z 和 cv2.circle 的代码保持原样不变)...
            # ...(剩下的计算 M, x, y, z 和 cv2.circle 的代码保持原样不变)...
            area = cv2.contourArea(cnt)
            if area < 50: # 忽略太小的噪点 (根据你的红点大小调整)
                continue

            # 计算红点质心
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])
                
                # 查表法：直接去深度图里拿这个像素的深度 (单位：毫米)
                depth_mm = depth_image[v, u]
                
                # 如果正中心没深度(比如反光)，可以取周围一小圈的平均值，这里简化处理
                if depth_mm == 0:
                    continue
                    
                # 转为米
                z = depth_mm / 1000.0
                
                # 终极反投影：像素 (u,v) -> 物理世界 (X,Y)
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                # 打包成 Pose 存入数组
                pose = Pose()
                pose.position = Point(x=x, y=y, z=z)
                pose_array_msg.poses.append(pose)
                
                # --- 在 Debug 图上画准星和数值 ---
                cv2.circle(color_image, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(color_image, f"P{i}: ({x:.2f}, {y:.2f}, {z:.2f})m", 
                            (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                self.get_logger().info(f"🎯 找到红点 {i+1}，坐标: X={x:.3f}, Y={y:.3f}, Z={z:.3f} 米")

        # 发布 3D 坐标组
        if len(pose_array_msg.poses) > 0:
            self.pose_pub.publish(pose_array_msg)
            
        # 发布 Debug 图像供 RViz 查看
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            debug_msg.header = color_msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RedDotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()