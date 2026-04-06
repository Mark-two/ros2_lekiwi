#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseArray, Pose, Point
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class RedDotDetectorNode(Node):
    def __init__(self):
        super().__init__('red_dot_detector')
        self.bridge = CvBridge()
        
        self.fx = self.fy = self.cx = self.cy = None
        
        # 存放经过平滑滤波后的重力向量
        self.gravity_vector = None 
        
        # 1. 订阅相机内参
        from rclpy.qos import qos_profile_sensor_data # 确保导入了 QoS (防幽灵BUG)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, qos_profile_sensor_data)
            
        # 2. 订阅相机的 IMU 话题
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu', self.imu_callback, qos_profile_sensor_data)

        # 3. 建立时间同步订阅器
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile_sensor_data)
            
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # 4. 发布器
        self.pose_pub = self.create_publisher(PoseArray, '/lekiwi/vision/red_dots_3d', 10)
        self.debug_pub = self.create_publisher(Image, '/lekiwi/vision/debug_image', 10)

        self.get_logger().info("🚀 极简流红点探测器 (全视角 HUD 显示版) 已启动！")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.destroy_subscription(self.info_sub)

    def imu_callback(self, msg):
        """ 提取重力向量并进行滤波平滑 """
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        current_accel = np.array([ax, ay, az])
        
        if self.gravity_vector is None:
            self.gravity_vector = current_accel
        else:
            alpha = 0.05 
            self.gravity_vector = (1.0 - alpha) * self.gravity_vector + alpha * current_accel

    def sync_callback(self, color_msg, depth_msg):
        if self.fx is None:
            return 

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception:
            return

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red1, upper_red1 = np.array([0, 213, 50]), np.array([8, 255, 255])
        lower_red2, upper_red2 = np.array([170, 213, 50]), np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.dilate(cv2.bitwise_or(mask1, mask2), np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours if 10 < cv2.contourArea(cnt) < 2000 and 
                          (4 * np.pi * (cv2.contourArea(cnt) / (cv2.arcLength(cnt, True)**2))) > 0.3]
        valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)[:2]

        pose_array_msg = PoseArray()
        pose_array_msg.header = color_msg.header
        
        pixel_coords = [] 
        camera_points = [] 

        # 2. 提取相机的原始 3D 坐标并将其写在对应的点旁边
        for i, cnt in enumerate(valid_contours):
            area = cv2.contourArea(cnt)
            if area < 50: continue

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                depth_mm = depth_image[v, u]
                if depth_mm == 0: continue
                    
                z_c = depth_mm / 1000.0
                x_c = (u - self.cx) * z_c / self.fx
                y_c = (v - self.cy) * z_c / self.fy

                camera_points.append(np.array([x_c, y_c, z_c]))

                pose = Pose()
                pose.position = Point(x=x_c, y=y_c, z=z_c)
                pose_array_msg.poses.append(pose)
                pixel_coords.append((u, v))
                
                # 💥 新增：在画面上标出每个单独红点的 (X, Y, Z) 原始坐标
                cv2.circle(color_image, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(color_image, f"P{i}: ({x_c:.3f}, {y_c:.3f}, {z_c:.3f})m", 
                            (u - 50, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # 3. 核心：利用 IMU 数据进行动态重力向量投影测距
        if len(camera_points) == 2 and self.gravity_vector is not None:
            vec_diff = camera_points[0] - camera_points[1]
            distance = np.linalg.norm(vec_diff)

            up_vec = self.gravity_vector / np.linalg.norm(self.gravity_vector)
            cam_right = np.array([1.0, 0.0, 0.0])
            level_right = cam_right - np.dot(cam_right, up_vec) * up_vec
            level_right = level_right / np.linalg.norm(level_right)
            level_forward = np.cross(up_vec, level_right)

            dz_true = abs(np.dot(vec_diff, up_vec))
            dx_true = abs(np.dot(vec_diff, level_right))
            dy_true = abs(np.dot(vec_diff, level_forward))

            self.get_logger().info(
                f"📏 [动态补偿] 直线: {distance:.3f}m | 左/右: {dx_true:.3f}m | 前/后: {dy_true:.3f}m | 高差: {dz_true:.3f}m"
            )

            # --- 画线标注 ---
            if len(pixel_coords) == 2:
                pt1, pt2 = pixel_coords[0], pixel_coords[1]
                cv2.line(color_image, pt1, pt2, (255, 0, 255), 2)
                mid_u, mid_v = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                
                # 💥 新增：把各个方向的距离排版好，全显示出来
                # 为了不挡住线，把文字稍微向右挪一点点
                text_x = mid_u + 10
                cv2.putText(color_image, f"Dist : {distance:.3f}m", 
                            (text_x, mid_v - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 2)
                cv2.putText(color_image, f"dX(L/R): {dx_true:.3f}m", 
                            (text_x, mid_v - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
                cv2.putText(color_image, f"dY(F/B): {dy_true:.3f}m", 
                            (text_x, mid_v + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
                cv2.putText(color_image, f"dZ(Up/Dn): {dz_true:.3f}m", 
                            (text_x, mid_v + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

        if len(pose_array_msg.poses) > 0:
            self.pose_pub.publish(pose_array_msg)
            
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            debug_msg.header = color_msg.header
            self.debug_pub.publish(debug_msg)
        except Exception:
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()