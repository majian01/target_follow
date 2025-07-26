#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from detector_pkg.msg import YOLOResults, Object
from alignment_config import AlignmentConfig, PresetConfigs, DebugUtils

FRAME_SIZE = (640, 480)  # 图像分辨率 (像素)

class CamFollowNode:
    def __init__(self, use_preset="balanced"):
        rospy.init_node('cam_follow', anonymous=False)
        
        # 加载配置
        if use_preset == "precise":
            self.config = PresetConfigs.get_precise_config()
        elif use_preset == "fast":
            self.config = PresetConfigs.get_fast_config()
        else:  # balanced
            self.config = PresetConfigs.get_balanced_config()
        
        # 打印配置信息
        self.config.print_config()
        
        # 图像相关参数
        self.mid_x = FRAME_SIZE[0] / 2  # 图像中心x
        self.target_center_point = self.mid_x  # 上一次目标的中心x
        self.target_box_width = 0  # 上一次目标的宽度

        # 控制参数
        self.speed_max = 0.45  # 最大速度
        self.back_speed = -0.15  # 后退速度
        self.t = 0.05  # 两张图片的时间间隔 (s)
        self.base_w = self.mid_x * 0.6  # 基准框宽度 (像素)
        self.bash_y = 1  # 基准距离 (m)

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
        
        # 对齐统计
        self.alignment_stats = {
            'total_frames': 0,
            'aligned_frames': 0,
            'avg_error': 0.0
        }
        
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
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        front_index = int((0 - angle_min) / angle_increment)
        front_distance = ranges[front_index] if 0 <= front_index < len(ranges) else float('inf')
        return front_distance

    def lidar_callback(self, msg):
        """
        雷达数据回调函数，检测障碍物
        """
        front_distance = self.get_front_distance(msg)
        if front_distance <= 0.50 and front_distance != 0:
            rospy.loginfo("Lidar detected obstacle too close, stopping robot.")
            self.count += 1
        else:
            self.count = max(0, self.count - 1)  # 逐渐减少计数
            
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
            
        gap = self.mid_x * 2
        cur_rect = None
        for rect in self.rect:
            cur_x = (rect[1][0] + rect[0][0]) / 2
            cur_gap = abs(cur_x - self.target_center_point)
            if cur_gap < gap:
                gap = cur_gap
                cur_rect = rect
        return cur_rect

    def calculate_precise_alignment_turn(self, target_center_x):
        """
        使用配置参数计算精确对齐的转向控制
        """
        # 计算目标中心与图像中心的偏差
        x_error = self.config.get_alignment_error(target_center_x, self.mid_x)
        
        # 如果已经对齐，不转向
        if self.config.is_aligned(target_center_x, self.mid_x):
            return 0
        
        # 使用比例控制计算转向速度
        turn_speed = x_error * self.config.TURN_KP
        
        # 限制最大转向速度
        turn_speed = max(-self.config.MAX_TURN_SPEED, 
                        min(self.config.MAX_TURN_SPEED, turn_speed))
        
        # 平滑转向命令
        smoothed_turn = (self.config.TURN_SMOOTH_FACTOR * self.last_turn_command + 
                        (1 - self.config.TURN_SMOOTH_FACTOR) * turn_speed)
        
        self.last_turn_command = smoothed_turn
        return smoothed_turn

    def update_alignment_stats(self, target_center_x):
        """
        更新对齐统计信息
        """
        self.alignment_stats['total_frames'] += 1
        
        if self.config.is_aligned(target_center_x, self.mid_x):
            self.alignment_stats['aligned_frames'] += 1
        
        # 计算平均误差
        current_error = abs(target_center_x - self.mid_x)
        total_frames = self.alignment_stats['total_frames']
        self.alignment_stats['avg_error'] = (
            (self.alignment_stats['avg_error'] * (total_frames - 1) + current_error) / total_frames
        )

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
            
            # 更新统计信息
            self.update_alignment_stats(self.target_center_point)
            
            # 使用精确对齐算法
            self.control_turn = self.calculate_precise_alignment_turn(self.target_center_point)
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
            # 目标太小，需要前进
            if self.control_speed <= 0.2:
                self.control_speed = 0.2
            
            # 根据对齐情况调整速度
            x_error = abs(self.target_center_point - self.mid_x)
            if x_error > self.config.SPEED_REDUCTION_THRESHOLD:
                # 目标偏离中心较多，降低速度以便更好地对齐
                max_allowed_speed = self.speed_max * self.config.SPEED_REDUCTION_FACTOR
            else:
                max_allowed_speed = self.speed_max
                
            # 计算期望速度
            distance = self.bash_y * (self.base_w / self.target_box_width) - self.bash_y
            speed = distance / self.t
            
            if speed > self.control_speed:
                if self.control_speed < max_allowed_speed:
                    self.control_speed += 0.05  # 匀速加速
            else:
                self.control_speed = min(speed, max_allowed_speed)

    def get_alignment_rate(self):
        """
        获取对齐成功率
        """
        if self.alignment_stats['total_frames'] == 0:
            return 0.0
        return (self.alignment_stats['aligned_frames'] / 
                self.alignment_stats['total_frames'] * 100)

    def shutdown(self):
        rospy.loginfo("Shutting down the cam_follow_node")
        
        # 打印最终统计信息
        if self.alignment_stats['total_frames'] > 0:
            rospy.loginfo("=== 对齐统计信息 ===")
            rospy.loginfo(f"总帧数: {self.alignment_stats['total_frames']}")
            rospy.loginfo(f"对齐帧数: {self.alignment_stats['aligned_frames']}")
            rospy.loginfo(f"对齐成功率: {self.get_alignment_rate():.1f}%")
            rospy.loginfo(f"平均误差: {self.alignment_stats['avg_error']:.2f} 像素")
            rospy.loginfo("=" * 20)
        
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
        last_stats_time = time.time()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            self.follow_target()
            
            # 每0.5秒打印一次状态信息
            if current_time - last_log_time > 0.5:
                last_log_time = current_time
                
                if self.target_box_width > 0:  # 有目标时才显示对齐信息
                    alignment_status = DebugUtils.log_alignment_status(
                        self.target_center_point, self.mid_x, self.config.ALIGN_TOLERANCE
                    )
                    quality = DebugUtils.calculate_alignment_quality(
                        self.target_center_point, self.mid_x, self.config.ALIGN_TOLERANCE
                    )
                    rospy.loginfo(f"速度: {self.control_speed:.2f}, 转向: {self.control_turn:.2f}, "
                                f"状态: {alignment_status}, 质量: {quality:.1f}%")
                else:
                    rospy.loginfo("等待目标检测...")
            
            # 每10秒打印一次统计信息
            if current_time - last_stats_time > 10.0:
                last_stats_time = current_time
                if self.alignment_stats['total_frames'] > 0:
                    rospy.loginfo(f"对齐成功率: {self.get_alignment_rate():.1f}%, "
                                f"平均误差: {self.alignment_stats['avg_error']:.2f}px")

            # 发布控制命令
            twist = Twist()
            twist.linear.x = self.control_speed
            twist.angular.z = self.control_turn
            self.vel_pub.publish(twist)
            
            rate.sleep()

if __name__ == "__main__":
    import sys
    
    # 支持命令行参数选择配置
    preset = "balanced"  # 默认配置
    if len(sys.argv) > 1:
        if sys.argv[1] in ["precise", "balanced", "fast"]:
            preset = sys.argv[1]
        else:
            rospy.logwarn(f"未知配置 '{sys.argv[1]}'，使用默认配置 'balanced'")
    
    rospy.loginfo(f"使用配置: {preset}")
    
    try:
        cam_follow_node = CamFollowNode(use_preset=preset)
        rospy.on_shutdown(cam_follow_node.shutdown)
        cam_follow_node.main_loop()
    except rospy.ROSInterruptException:
        pass