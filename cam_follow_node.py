#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from detector_pkg.msg import YOLOResults, Object

FRAME_SIZE = (640, 480)  # 图像分辨率 (像素)

class CamFollowNode:
    def __init__(self):
        rospy.init_node('cam_follow', anonymous=False)
        # 图像相关参数
        self.mid_x = FRAME_SIZE[0] / 2  # 图像中心x坐标 (320)
        self.mid_y = FRAME_SIZE[1] / 2  # 图像中心y坐标 (240)
        self.target_center_point = self.mid_x  # 上一次目标的中心x
        self.target_box_width = 0  # 上一次目标的宽度

        # 控制参数
        self.speed_max = 0.45  # 最大速度
        self.back_speed = -0.15  # 后退速度
        self.t = 0.05  # 两张图片的时间间隔 (s)
        self.half_rad = 0.7  # 半角弧度
        self.base_w = self.mid_x * 0.6  # 基准框宽度 (像素)
        self.bash_y = 1  # 基准距离 (m)

        # 对齐控制参数 - 改进的转向阈值
        self.alignment_tolerance = 20  # 允许的偏差像素，目标在此范围内认为已对齐
        self.turn_gain = 0.5  # 转向增益，控制转向的敏感度
        self.max_turn_speed = 1.0  # 最大转向速度
        
        # 原有的转向和前进的阈值
        self.x_l_min = self.mid_x * 0.8  # 向左转向阈值
        self.x_r_min = self.mid_x * 0.8  # 向右转向阈值
        self.x_l_max = self.mid_x * 1.0  # 向左转向阈值
        self.x_r_max = self.mid_x * 1.0  # 向右转向阈值
        self.w_b_min = self.mid_x * 0.8  # 小车后退阈值
        self.w_f_min = self.mid_x * 0.2  # 小车前进阈值
        self.stop_flag = False  # 停止标志
        self.count = 0

        # 控制命令
        self.control_speed = 0  # 控制前后速度
        self.control_turn = 0  # 控制转向速度
        self.rect = []  # 存储目标框
        
        # ROS 相关
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.yolo_sub = rospy.Subscriber('/yolo/results', YOLOResults, self.yolo_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def process_yolo_results(self, msg):
        """
        处理 YOLO 检测结果，将检测到的目标转换为矩形框
        """
        self.rect = []  # 清空之前的检测结果
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
        雷达数据回调函数
        """
        rospy.loginfo("Received Lidar data")
        front_distance = self.get_front_distance(msg)
        rospy.loginfo(f"Front distance: {front_distance:.2f} m")
        if front_distance <= 0.50 and front_distance != 0:
            rospy.loginfo("Lidar detected obstacle too close, stopping robot.")
            self.count += 1
        if self.count > 5:
            self.stop_flag = True

    def find_target_rect(self):
        """
        查找目标框，返回与上一次检测位置差值最小的目标框
        """
        assert len(self.rect) > 0, "Rectangles are empty"
        gap = self.mid_x * 2  # 寻找和上一个点差值最小的点
        cur_rect = []
        for rect in self.rect:
            cur_x = (rect[1][0] + rect[0][0]) / 2
            cur_gap = abs(cur_x - self.target_center_point)
            if cur_gap < gap:
                gap = cur_gap
                cur_rect = rect
        return cur_rect

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
            self.target_center_point = (target[1][0] + target[0][0]) / 2  # 计算目标框中心x坐标
            self.target_box_width = target[1][0] - target[0][0]  # 目标框宽度
            self.calculate_turn_for_alignment()
            self.calculate_speed()

    def calculate_turn_for_alignment(self):
        """
        改进的转向计算函数，用于让目标中心与摄像头中心对齐
        """
        if self.stop_flag:
            self.control_turn = 0
            return
            
        # 计算目标中心与图像中心的偏差
        error_x = self.target_center_point - self.mid_x
        
        # 如果偏差在容忍范围内，不进行转向
        if abs(error_x) <= self.alignment_tolerance:
            self.control_turn = 0
            rospy.loginfo(f"Target aligned! Error: {error_x:.2f} pixels")
            return
        
        # 计算转向速度，使用比例控制
        # 负值表示目标在左侧，需要向左转；正值表示目标在右侧，需要向右转
        turn_speed = -error_x * self.turn_gain / self.mid_x
        
        # 限制最大转向速度
        if abs(turn_speed) > self.max_turn_speed:
            turn_speed = self.max_turn_speed if turn_speed > 0 else -self.max_turn_speed
        
        self.control_turn = turn_speed
        
        # 调试信息
        rospy.loginfo(f"Target center: {self.target_center_point:.2f}, Image center: {self.mid_x:.2f}")
        rospy.loginfo(f"Error: {error_x:.2f} pixels, Turn speed: {turn_speed:.3f}")

    def calculate_turn(self):
        """
        原有的转向计算函数（保留作为备用）
        """
        if self.x_l_min <= self.target_center_point <= self.x_r_min or self.stop_flag: # 处于中间区域或停止标志
            # 不转向
            self.control_turn = 0
        elif self.target_center_point < self.x_l_min: # 处于左侧区域
            # 向左转
            turn_rad = self.half_rad * (self.x_l_min - self.target_center_point) / self.mid_x  # 转动角度
            self.control_turn = turn_rad / self.t
        else:
            # 向右转
            turn_rad = self.half_rad * (self.x_r_min - self.target_center_point) / self.mid_x
            self.control_turn = turn_rad / self.t

        self.control_turn /= 4

    def calculate_speed(self):
        """
        根据目标框宽度和距离，计算小车的前进速度
        """
        if self.w_f_min <= self.target_box_width <= self.w_b_min or self.stop_flag:
            # 不运动
            self.control_speed = 0
        elif self.target_box_width > self.w_b_min:
            # 后退
            self.control_speed = self.back_speed
        else:
            if self.control_speed <= 0.2:
                self.control_speed = 0.2
            # 小车前进
            if self.target_center_point <= self.x_l_min or self.target_center_point >= self.x_r_max:
                pass  # 处于边缘，不让车太快
            else:
                distance = self.bash_y * (self.base_w / self.target_box_width) - self.bash_y  # 距离计算
                speed = distance / self.t  # 计算速度
                if speed > self.control_speed:
                    if self.control_speed < self.speed_max:
                        self.control_speed += 0.05  # 匀速加速
                else:
                    self.control_speed = speed

    def shutdown(self):
        """
        关闭节点时的清理工作
        """
        rospy.loginfo("Shutting down the cam_follow_node")
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)

    def main_loop(self):
        """
        主循环，控制机器人根据 YOLO 检测结果跟随目标
        """
        rate = rospy.Rate(20)  # 20Hz控制频率
        last_log_time = time.time()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # 执行目标跟随逻辑
            self.follow_target()
            
            # 定期打印调试信息
            if current_time - last_log_time > 0.5:  # 每0.5秒打印一次
                last_log_time = current_time
                rospy.loginfo(f"Control Speed: {self.control_speed:.2f}, Control Turn: {self.control_turn:.2f}")
                if self.rect:
                    rospy.loginfo(f"Detected {len(self.rect)} targets")

            # 发布控制命令
            twist = Twist()
            twist.linear.x = self.control_speed
            twist.angular.z = self.control_turn
            self.vel_pub.publish(twist)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        cam_follow_node = CamFollowNode()
        rospy.on_shutdown(cam_follow_node.shutdown)
        cam_follow_node.main_loop()
    except rospy.ROSInterruptException:
        pass