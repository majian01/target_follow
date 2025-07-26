#!/usr/bin/env python
# coding=utf-8
"""
基础转向测试脚本：测试机器人是否能够正常左转和右转
"""
import rospy
from geometry_msgs.msg import Twist
import time

def test_basic_turning():
    """
    测试机器人的基础转向功能
    """
    rospy.init_node('basic_turn_test', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # 等待发布器连接
    rospy.sleep(1)
    
    print("开始基础转向测试...")
    print("测试期间请观察机器人的转向行为")
    
    test_cases = [
        ("LEFT TURN (positive angular.z)", 0.5, 3),   # 左转3秒
        ("STOP", 0.0, 2),                             # 停止2秒
        ("RIGHT TURN (negative angular.z)", -0.5, 3), # 右转3秒
        ("STOP", 0.0, 2),                             # 停止2秒
        ("SLOW LEFT", 0.2, 2),                        # 慢速左转
        ("STOP", 0.0, 1),
        ("SLOW RIGHT", -0.2, 2),                      # 慢速右转
        ("STOP", 0.0, 1),
    ]
    
    for description, angular_speed, duration in test_cases:
        print(f"\n执行: {description} (angular.z = {angular_speed}, {duration}s)")
        
        # 发送转向命令
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            vel_pub.publish(twist)
            rospy.sleep(0.1)
    
    # 最终停止
    print("\n测试完成，停止机器人")
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    vel_pub.publish(twist)

if __name__ == "__main__":
    try:
        test_basic_turning()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except Exception as e:
        print(f"测试出错: {e}")
        # 确保机器人停止
        try:
            vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            twist = Twist()
            vel_pub.publish(twist)
        except:
            pass