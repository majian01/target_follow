#!/usr/bin/env python
# coding=utf-8
"""
测试脚本：用于验证转向逻辑是否正确
模拟不同的目标位置，检查转向方向是否正确
"""

FRAME_SIZE = (640, 480)

class AlignmentTester:
    def __init__(self):
        self.mid_x = FRAME_SIZE[0] / 2  # 320
        self.alignment_tolerance = 15
        self.turn_gain = 1.0
        self.max_turn_speed = 0.8
    
    def calculate_turn_for_alignment(self, target_center_point):
        """
        测试转向计算函数
        """
        # 计算目标中心与图像中心的偏差
        error_x = target_center_point - self.mid_x
        
        # 如果偏差在容忍范围内，不进行转向
        if abs(error_x) <= self.alignment_tolerance:
            print(f"Target aligned! Error: {error_x:.2f} pixels")
            return 0
        
        # 计算转向速度
        turn_speed = -error_x * self.turn_gain / self.mid_x
        
        # 限制最大转向速度
        if abs(turn_speed) > self.max_turn_speed:
            turn_speed = self.max_turn_speed if turn_speed > 0 else -self.max_turn_speed
        
        return turn_speed
    
    def test_alignment(self):
        """
        测试不同位置的转向逻辑
        """
        print("=== 转向逻辑测试 ===")
        print(f"图像分辨率: {FRAME_SIZE}")
        print(f"图像中心: {self.mid_x}")
        print(f"容忍范围: ±{self.alignment_tolerance} 像素")
        print()
        
        # 测试用例: [目标位置, 期望转向方向]
        test_cases = [
            (100, "LEFT"),    # 目标在左侧，应该左转(positive angular.z)
            (200, "LEFT"),    # 目标在左侧，应该左转
            (300, "LEFT"),    # 目标稍微在左侧，应该左转
            (310, "NONE"),    # 目标接近中心，不转向
            (320, "NONE"),    # 目标在中心，不转向
            (330, "NONE"),    # 目标接近中心，不转向
            (340, "RIGHT"),   # 目标稍微在右侧，应该右转(negative angular.z)
            (420, "RIGHT"),   # 目标在右侧，应该右转
            (540, "RIGHT"),   # 目标在右侧，应该右转
        ]
        
        print("测试结果:")
        print("目标位置 | 偏差(px) | 转向速度 | 转向方向 | 期望方向 | 结果")
        print("-" * 70)
        
        for target_pos, expected_dir in test_cases:
            turn_speed = self.calculate_turn_for_alignment(target_pos)
            error_x = target_pos - self.mid_x
            
            if abs(turn_speed) < 0.001:
                actual_dir = "NONE"
            elif turn_speed > 0:
                actual_dir = "LEFT"
            else:
                actual_dir = "RIGHT"
            
            result = "✓" if actual_dir == expected_dir else "✗"
            
            print(f"{target_pos:8.0f} | {error_x:8.2f} | {turn_speed:8.3f} | {actual_dir:8s} | {expected_dir:8s} | {result}")
        
        print()
        print("说明:")
        print("- LEFT: 正值 angular.z (逆时针转动)")
        print("- RIGHT: 负值 angular.z (顺时针转动)")
        print("- NONE: 目标已对齐，不需要转向")

if __name__ == "__main__":
    tester = AlignmentTester()
    tester.test_alignment()