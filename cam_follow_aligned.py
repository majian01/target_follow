#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from detector_pkg.msg import YOLOResults, Object
import math

FRAME_SIZE = (640, 480)  # 图像分辨率 (像素)

class CamFollowAlignedNode:
    def __init__(self):
        rospy.init_node('cam_follow_aligned', anonymous=False)
        # 图像相关参数
        self.mid_x = FRAME_SIZE[0] / 2  # 图像中心x (320像素)
        self.mid_y = FRAME_SIZE[1] / 2  # 图像中心y (240像素)
        self.target_center_point = self.mid_x  # 上一次目标的中心x
        self.target_box_width = 0  # 上一次目标的宽度

        # 控制参数
        self.speed_max = 0.45  # 最大速度
        self.back_speed = -0.15  # 后退速度
        self.t = 0.05  # 两张图片的时间间隔 (s)
        self.base_w = self.mid_x * 0.6  # 基准框宽度 (像素)
        self.bash_y = 1  # 基准距离 (m)

        # 精确对齐的控制参数
        self.align_tolerance = 10  # 对齐容差 (像素)，目标中心在此范围内认为已对齐
        self.max_turn_speed = 0.8  # 最大转向速度 (rad/s)
        self.turn_kp = 0.003  # 转向比例控制系数，调小可以使转向更平滑
        
        # 前进控制的阈值
        self.w_b_min = self.mid_x * 0.8  # 小车后退阈值
        self.w_f_min = self.mid_x * 0.2  # 小车前进阈值
        self.stop_flag = False  # 停止标志
        self.count = 0

        # 控制命令
        self.control_speed = 0  # 控制前后速度
        self.control_turn = 0  # 控制转向速度
        self.rect = []  # 存储目标框
        
        # 平滑控制相关
        self.last_turn_command = 0  # 上一次转向命令
        self.turn_smooth_factor = 0.7  # 转向平滑因子 (0-1)
        
        # ROS 相关
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.yolo_sub = rospy.Subscriber('/yolo/results', YOLOResults, self.yolo_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def process_yolo_results(self, msg):
        """
        处理 YOLO 检测结果，将检测到的目标转换为矩形框
        """
        self.rect = []  # 清空之前的结果
        for object in msg.results:
            point_list = [[],[]]
            point_list[0].append(object.x_min)
            point_list[0].append(object.y_min)
            point_list[1].append(object.x_max)
            point_list[1].append(object.y_max)
            self.rect.append(point_list)
    
    def yolo_callback(self, msg):
        """
        YOLO检测回调函数，处理检测结果
        """
        if msg.results is None or len(msg.results) == 0:
            self.rect = []
        else:
            self.process_yolo_results(msg)
        
    def get_front_distance(self, msg):
        """
        获取前方距离
        """
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        front_index = int((0 - angle_min) / angle_increment)
        front_distance = ranges[front_index] if 0 <= front_index < len(ranges) else float('inf')
        return front_distance

    def lidar_callback(self, msg):
        """
        雷达数据回调函数，检测前方障碍物
        """
        front_distance = self.get_front_distance(msg)
        rospy.loginfo(f"Front distance: {front_distance:.2f} m")
        if front_distance <= 0.50 and front_distance != 0:
            rospy.loginfo("Lidar detected obstacle too close, stopping robot.")
            self.count += 1
        else:
            self.count = 0  # 重置计数器
            
        if self.count > 5:
            self.stop_flag = True
        else:
            self.stop_flag = False

    def find_target_rect(self):
        """
        查找目标框，返回与上一次检测位置差值最小的目标框
        """
        if not self.rect:
            return None
            
        gap = self.mid_x * 2  # 寻找和上一个点差值最小的点
        cur_rect = None
        for rect in self.rect:
            cur_x = (rect[1][0] + rect[0][0]) / 2
            cur_gap = abs(cur_x - self.target_center_point)
            if cur_gap < gap:
                gap = cur_gap
                cur_rect = rect
        return cur_rect

    def calculate_alignment_turn(self, target_center_x):
        """
        计算精确对齐的转向控制
        """
        # 计算目标中心与图像中心的偏差
        x_error = target_center_x - self.mid_x
        
        # 如果偏差在容差范围内，不转向
        if abs(x_error) <= self.align_tolerance:
            return 0
        
        # 使用比例控制计算转向速度
        turn_speed = x_error * self.turn_kp
        
        # 限制最大转向速度
        turn_speed = max(-self.max_turn_speed, min(self.max_turn_speed, turn_speed))
        
        # 平滑转向命令
        smoothed_turn = (self.turn_smooth_factor * self.last_turn_command + 
                        (1 - self.turn_smooth_factor) * turn_speed)
        
        self.last_turn_command = smoothed_turn
        return smoothed_turn

    def follow_target(self):
        """
        根据目标框的位置，控制小车的速度和转向
        """
        if not self.rect:
            self.target_center_point = self.mid_x  # 没有目标，恢复初始位置
            self.target_box_width = 0
            self.control_turn = 0
            self.control_speed = 0
            return

        if len(self.rect) == 1:  # 只有一个框
            target = self.rect[0]
        else:  # 多个框，选择差值最小的目标
            target = self.find_target_rect()

        if target:
            self.target_center_point = (target[1][0] + target[0][0]) / 2  # 计算目标框中心
            self.target_box_width = target[1][0] - target[0][0]  # 目标框宽度
            
            # 使用新的精确对齐算法
            self.control_turn = self.calculate_alignment_turn(self.target_center_point)
            self.calculate_speed()

    def calculate_speed(self):
        """
        根据目标框宽度和距离，计算小车的前进速度
        """
        if self.stop_flag:
            self.control_speed = 0
            return
            
        if self.w_f_min <= self.target_box_width <= self.w_b_min:
            # 目标大小合适，不运动
            self.control_speed = 0
        elif self.target_box_width > self.w_b_min:
            # 目标太大，后退
            self.control_speed = self.back_speed
        else:
            # 目标太小，前进
            if self.control_speed <= 0.2:
                self.control_speed = 0.2
            
            # 如果目标不在中心区域，降低前进速度以便更好地对齐
            x_error = abs(self.target_center_point - self.mid_x)
            if x_error > self.align_tolerance * 2:
                # 目标偏离中心较多，降低速度
                max_speed = self.speed_max * 0.5
            else:
                max_speed = self.speed_max
            
            distance = self.bash_y * (self.base_w / self.target_box_width) - self.bash_y  # 距离计算
            speed = distance / self.t  # 计算速度
            
            if speed > self.control_speed:
                if self.control_speed < max_speed:
                    self.control_speed += 0.05  # 匀速加速
            else:
                self.control_speed = min(speed, max_speed)

    def shutdown(self):
        """
        关闭节点时停止机器人
        """
        rospy.loginfo("Shutting down the cam_follow_aligned_node")
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)

    def main_loop(self):
        """
        主循环，控制机器人根据 YOLO 检测结果跟随目标并保持对齐
        """
        rate = rospy.Rate(20)  # 20Hz控制频率
        last_log_time = time.time()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            self.follow_target()
            
            # 每0.2秒打印一次状态信息
            if current_time - last_log_time > 0.2:
                last_log_time = current_time
                x_error = abs(self.target_center_point - self.mid_x)
                alignment_status = "ALIGNED" if x_error <= self.align_tolerance else f"ERROR: {x_error:.1f}px"
                rospy.loginfo(f"Speed: {self.control_speed:.2f}, Turn: {self.control_turn:.2f}, Status: {alignment_status}")

            # 发布控制命令
            twist = Twist()
            twist.linear.x = self.control_speed
            twist.angular.z = self.control_turn
            self.vel_pub.publish(twist)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        cam_follow_node = CamFollowAlignedNode()
        rospy.on_shutdown(cam_follow_node.shutdown)
        cam_follow_node.main_loop()
    except rospy.ROSInterruptException:
        pass