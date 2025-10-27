#!/usr/bin/env python
# coding=utf-8
"""
障碍检测测试脚本：测试前方50cm障碍检测功能
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleDetectionTester:
    def __init__(self):
        rospy.init_node('obstacle_detection_test', anonymous=True)
        
        # 障碍检测参数（与主程序一致）
        self.obstacle_distance_threshold = 0.50  # 50cm
        self.obstacle_detection_angle = 30  # 30度检测范围
        self.obstacle_count_threshold = 3
        self.obstacle_recovery_distance = 0.70  # 70cm恢复距离
        
        # 状态变量
        self.obstacle_detected = False
        self.obstacle_count = 0
        self.min_front_distance = float('inf')
        
        # ROS订阅和发布
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        rospy.loginfo("障碍检测测试启动...")
        rospy.loginfo(f"检测距离阈值: {self.obstacle_distance_threshold}m")
        rospy.loginfo(f"检测角度范围: ±{self.obstacle_detection_angle/2}度")
        rospy.loginfo(f"恢复距离: {self.obstacle_recovery_distance}m")
    
    def get_front_distance_range(self, msg):
        """
        获取前方扇形区域内的最小距离
        """
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        
        # 计算检测角度范围
        detection_angle_rad = self.obstacle_detection_angle * math.pi / 180.0
        half_angle = detection_angle_rad / 2.0
        
        # 计算前方检测区域的索引范围
        center_index = int((0 - angle_min) / angle_increment)
        angle_range = int(half_angle / angle_increment)
        
        start_index = max(0, center_index - angle_range)
        end_index = min(len(ranges), center_index + angle_range + 1)
        
        # 找到检测区域内的最小距离
        min_distance = float('inf')
        valid_readings = 0
        distances = []
        
        for i in range(start_index, end_index):
            if 0 < ranges[i] < 10.0:  # 过滤无效数据
                distances.append(ranges[i])
                min_distance = min(min_distance, ranges[i])
                valid_readings += 1
        
        if valid_readings == 0:
            min_distance = float('inf')
            
        return min_distance, valid_readings, distances
    
    def detect_obstacle(self, min_distance):
        """
        检测障碍物并更新状态
        """
        self.min_front_distance = min_distance
        
        # 检测障碍物
        if min_distance <= self.obstacle_distance_threshold and min_distance != float('inf'):
            self.obstacle_count += 1
            if self.obstacle_count >= self.obstacle_count_threshold:
                if not self.obstacle_detected:
                    rospy.logwarn("=" * 50)
                    rospy.logwarn(f"🚨 OBSTACLE DETECTED! 🚨")
                    rospy.logwarn(f"Distance: {min_distance:.2f}m")
                    rospy.logwarn(f"STOPPING ROBOT!")
                    rospy.logwarn("=" * 50)
                    # 发送停止命令
                    self.stop_robot()
                self.obstacle_detected = True
        else:
            # 检查是否可以恢复
            if min_distance >= self.obstacle_recovery_distance or min_distance == float('inf'):
                if self.obstacle_detected:
                    rospy.loginfo("=" * 50)
                    rospy.loginfo(f"✅ Obstacle cleared!")
                    rospy.loginfo(f"Distance: {min_distance:.2f}m")
                    rospy.loginfo(f"Robot can resume")
                    rospy.loginfo("=" * 50)
                self.obstacle_detected = False
                self.obstacle_count = 0
    
    def stop_robot(self):
        """
        发送停止命令
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
    
    def lidar_callback(self, msg):
        """
        雷达回调函数
        """
        min_distance, valid_readings, distances = self.get_front_distance_range(msg)
        
        # 检测障碍物
        self.detect_obstacle(min_distance)
        
        # 定期打印状态
        if hasattr(self, '_callback_count'):
            self._callback_count += 1
        else:
            self._callback_count = 0
            
        if self._callback_count % 10 == 0:  # 每10次回调打印一次
            status = "🚨 OBSTACLE" if self.obstacle_detected else "✅ CLEAR"
            rospy.loginfo(f"Status: {status} | Min Distance: {min_distance:.2f}m | Valid readings: {valid_readings}")
            
            if distances and len(distances) > 0:
                avg_distance = sum(distances) / len(distances)
                rospy.loginfo(f"Front scan: min={min_distance:.2f}m, avg={avg_distance:.2f}m, count={len(distances)}")

def main():
    try:
        tester = ObstacleDetectionTester()
        
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("障碍检测测试运行中...")
        rospy.loginfo("请在机器人前方50cm内放置障碍物进行测试")
        rospy.loginfo("观察终端输出的检测结果")
        rospy.loginfo("按 Ctrl+C 停止测试")
        rospy.loginfo("=" * 60 + "\n")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
    except Exception as e:
        rospy.logerr(f"测试出错: {e}")
    finally:
        # 确保机器人停止
        try:
            twist = Twist()
            vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            vel_pub.publish(twist)
            rospy.loginfo("机器人已停止")
        except:
            pass

if __name__ == "__main__":
    main()