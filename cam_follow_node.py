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
        
        # 障碍检测参数
        self.obstacle_distance_threshold = 0.50  # 障碍检测距离阈值 (m)
        self.obstacle_detection_angle = 30  # 前方检测角度范围 (度)
        self.obstacle_count_threshold = 3  # 连续检测到障碍的次数阈值
        self.obstacle_recovery_distance = 0.70  # 障碍消失恢复距离 (m)

        # 对齐控制参数 - 改进的转向阈值
        self.alignment_tolerance = 15  # 允许的偏差像素，目标在此范围内认为已对齐
        self.turn_gain = 1.0  # 转向增益，控制转向的敏感度 (增大以提高响应)
        self.max_turn_speed = 0.8  # 最大转向速度 (降低以防止过快转动)
        
        # 原有的转向和前进的阈值
        self.x_l_min = self.mid_x * 0.8  # 向左转向阈值
        self.x_r_min = self.mid_x * 0.8  # 向右转向阈值
        self.x_l_max = self.mid_x * 1.0  # 向左转向阈值
        self.x_r_max = self.mid_x * 1.0  # 向右转向阈值
        self.w_b_min = self.mid_x * 0.8  # 小车后退阈值
        self.w_f_min = self.mid_x * 0.2  # 小车前进阈值
        self.stop_flag = False  # 停止标志
        self.obstacle_detected = False  # 障碍检测标志
        self.obstacle_count = 0  # 连续检测到障碍的次数
        self.min_front_distance = float('inf')  # 当前最小前方距离

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
        
    def get_front_distance_range(self, msg):
        """
        获取前方扇形区域内的最小距离
        """
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        
        # 计算检测角度范围
        detection_angle_rad = self.obstacle_detection_angle * 3.14159 / 180.0
        half_angle = detection_angle_rad / 2.0
        
        # 计算前方检测区域的索引范围
        center_index = int((0 - angle_min) / angle_increment)
        angle_range = int(half_angle / angle_increment)
        
        start_index = max(0, center_index - angle_range)
        end_index = min(len(ranges), center_index + angle_range + 1)
        
        # 找到检测区域内的最小距离
        min_distance = float('inf')
        valid_readings = 0
        
        for i in range(start_index, end_index):
            if 0 < ranges[i] < 10.0:  # 过滤无效数据
                min_distance = min(min_distance, ranges[i])
                valid_readings += 1
        
        # 如果没有有效读数，返回无穷大
        if valid_readings == 0:
            min_distance = float('inf')
            
        return min_distance, valid_readings

    def detect_obstacle(self, min_distance):
        """
        检测障碍物并更新停止状态
        """
        # 更新最小距离
        self.min_front_distance = min_distance
        
        # 检测障碍物
        if min_distance <= self.obstacle_distance_threshold and min_distance != float('inf'):
            self.obstacle_count += 1
            if self.obstacle_count >= self.obstacle_count_threshold:
                if not self.obstacle_detected:
                    rospy.logwarn(f"OBSTACLE DETECTED! Distance: {min_distance:.2f}m - STOPPING ROBOT")
                self.obstacle_detected = True
                self.stop_flag = True
        else:
            # 检查是否可以恢复
            if min_distance >= self.obstacle_recovery_distance or min_distance == float('inf'):
                if self.obstacle_detected:
                    rospy.loginfo(f"Obstacle cleared! Distance: {min_distance:.2f}m - RESUMING")
                self.obstacle_detected = False
                self.stop_flag = False
                self.obstacle_count = 0

    def lidar_callback(self, msg):
        """
        雷达数据回调函数 - 改进的障碍检测
        """
        # 获取前方扇形区域的最小距离
        min_distance, valid_readings = self.get_front_distance_range(msg)
        
        # 检测障碍物
        self.detect_obstacle(min_distance)
        
        # 每5次回调打印一次状态信息
        if hasattr(self, '_lidar_callback_count'):
            self._lidar_callback_count += 1
        else:
            self._lidar_callback_count = 0
            
        if self._lidar_callback_count % 5 == 0:
            status = "OBSTACLE DETECTED" if self.obstacle_detected else "CLEAR"
            rospy.loginfo(f"Lidar: min_dist={min_distance:.2f}m, readings={valid_readings}, status={status}")

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
        # ROS约定: 正值angular.z = 逆时针(左转), 负值angular.z = 顺时针(右转)
        # error_x > 0: 目标在右侧，需要右转(负的angular.z)
        # error_x < 0: 目标在左侧，需要左转(正的angular.z)
        turn_speed = -error_x * self.turn_gain / self.mid_x
        
        # 限制最大转向速度
        if abs(turn_speed) > self.max_turn_speed:
            turn_speed = self.max_turn_speed if turn_speed > 0 else -self.max_turn_speed
        
        self.control_turn = turn_speed
        
        # 详细调试信息
        direction = "LEFT" if turn_speed > 0 else "RIGHT"
        rospy.loginfo(f"=== ALIGNMENT DEBUG ===")
        rospy.loginfo(f"Target center: {self.target_center_point:.2f}, Image center: {self.mid_x:.2f}")
        rospy.loginfo(f"Error: {error_x:.2f} pixels")
        rospy.loginfo(f"Turn speed: {turn_speed:.3f} rad/s ({direction})")
        rospy.loginfo(f"Turn gain: {self.turn_gain}, Max speed: {self.max_turn_speed}")
        
        if error_x > 0:
            rospy.loginfo("Target is on RIGHT side - turning RIGHT (negative angular.z)")
        else:
            rospy.loginfo("Target is on LEFT side - turning LEFT (positive angular.z)")
        rospy.loginfo("=======================") 

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
        # 优先检查障碍物 - 如果检测到障碍物，立即停止
        if self.obstacle_detected or self.stop_flag:
            self.control_speed = 0
            if self.obstacle_detected:
                rospy.logwarn(f"Speed blocked by obstacle at {self.min_front_distance:.2f}m")
            return
            
        # 正常的速度控制逻辑
        if self.w_f_min <= self.target_box_width <= self.w_b_min:
            # 目标距离合适，不运动
            self.control_speed = 0
        elif self.target_box_width > self.w_b_min:
            # 目标太近，后退（但要检查后方是否安全）
            # 注意：这里可以考虑添加后方障碍检测
            self.control_speed = self.back_speed
        else:
            # 目标较远，前进
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
                    
            # 根据前方距离调整速度 - 距离越近，速度越慢
            if self.min_front_distance < 1.5 and self.min_front_distance != float('inf'):
                distance_factor = max(0.1, (self.min_front_distance - self.obstacle_distance_threshold) / 
                                    (1.5 - self.obstacle_distance_threshold))
                self.control_speed *= distance_factor
                rospy.loginfo(f"Speed reduced due to proximity: factor={distance_factor:.2f}, new_speed={self.control_speed:.2f}")

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
                
                # 显示控制状态
                obstacle_status = "BLOCKED" if self.obstacle_detected else "CLEAR"
                rospy.loginfo(f"=== CONTROL STATUS ===")
                rospy.loginfo(f"Speed: {self.control_speed:.2f}, Turn: {self.control_turn:.2f}")
                rospy.loginfo(f"Obstacle: {obstacle_status} (dist: {self.min_front_distance:.2f}m)")
                if self.rect:
                    rospy.loginfo(f"Targets detected: {len(self.rect)}")
                else:
                    rospy.loginfo("No targets detected")
                rospy.loginfo("======================")
                

            # 发布控制命令
            twist = Twist()
            twist.linear.x = self.control_speed
            twist.angular.z = self.control_turn
            self.vel_pub.publish(twist)
            
            # 监控转向命令发送状态
            if abs(self.control_turn) > 0.01:  # 只在有显著转向时记录
                turn_direction = "LEFT" if self.control_turn > 0 else "RIGHT"
                rospy.loginfo(f"PUBLISHING: angular.z = {self.control_turn:.3f} ({turn_direction})")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        cam_follow_node = CamFollowNode()
        rospy.on_shutdown(cam_follow_node.shutdown)
        cam_follow_node.main_loop()
    except rospy.ROSInterruptException:
        pass