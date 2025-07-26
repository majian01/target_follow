# 摄像头对齐配置参数
# 这个文件包含了用于精确对齐摄像头中点和检测目标中心的所有参数

class AlignmentConfig:
    """
    摄像头对齐配置类
    调整这些参数可以优化对齐效果
    """
    
    # 图像参数
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    
    # 对齐精度参数
    ALIGN_TOLERANCE = 5  # 对齐容差 (像素)
    # 值越小，对齐越精确，但可能导致震荡
    # 建议范围: 3-15像素
    
    # 转向控制参数
    TURN_KP = 0.004  # 转向比例控制系数
    # 值越大，转向响应越快，但可能过冲
    # 建议范围: 0.001-0.008
    
    MAX_TURN_SPEED = 0.6  # 最大转向速度 (rad/s)
    # 限制最大转向速度，防止机器人转向过快
    # 建议范围: 0.3-1.0 rad/s
    
    # 平滑控制参数
    TURN_SMOOTH_FACTOR = 0.7  # 转向平滑因子 (0-1)
    # 值越大，转向越平滑，但响应变慢
    # 建议范围: 0.5-0.9
    
    # 速度控制参数
    SPEED_REDUCTION_THRESHOLD = 15  # 偏离中心多少像素时开始降速
    SPEED_REDUCTION_FACTOR = 0.6  # 偏离中心时的速度系数
    
    @classmethod
    def get_image_center(cls):
        """获取图像中心坐标"""
        return cls.FRAME_WIDTH / 2, cls.FRAME_HEIGHT / 2
    
    @classmethod
    def is_aligned(cls, target_x, center_x):
        """判断目标是否与中心对齐"""
        return abs(target_x - center_x) <= cls.ALIGN_TOLERANCE
    
    @classmethod
    def get_alignment_error(cls, target_x, center_x):
        """获取对齐误差"""
        return target_x - center_x
    
    @classmethod
    def print_config(cls):
        """打印当前配置"""
        print("=== 摄像头对齐配置 ===")
        print(f"图像尺寸: {cls.FRAME_WIDTH}x{cls.FRAME_HEIGHT}")
        print(f"对齐容差: {cls.ALIGN_TOLERANCE} 像素")
        print(f"转向系数: {cls.TURN_KP}")
        print(f"最大转向速度: {cls.MAX_TURN_SPEED} rad/s")
        print(f"平滑因子: {cls.TURN_SMOOTH_FACTOR}")
        print("=" * 25)

# 预设配置
class PresetConfigs:
    """预设的配置方案"""
    
    @staticmethod
    def get_precise_config():
        """精确对齐配置 - 高精度，响应较慢"""
        config = AlignmentConfig()
        config.ALIGN_TOLERANCE = 3
        config.TURN_KP = 0.002
        config.MAX_TURN_SPEED = 0.4
        config.TURN_SMOOTH_FACTOR = 0.8
        return config
    
    @staticmethod
    def get_balanced_config():
        """平衡配置 - 精度和响应的平衡"""
        config = AlignmentConfig()
        config.ALIGN_TOLERANCE = 5
        config.TURN_KP = 0.004
        config.MAX_TURN_SPEED = 0.6
        config.TURN_SMOOTH_FACTOR = 0.7
        return config
    
    @staticmethod
    def get_fast_config():
        """快速响应配置 - 响应快，精度稍低"""
        config = AlignmentConfig()
        config.ALIGN_TOLERANCE = 8
        config.TURN_KP = 0.006
        config.MAX_TURN_SPEED = 0.8
        config.TURN_SMOOTH_FACTOR = 0.6
        return config

# 调试工具
class DebugUtils:
    """调试辅助工具"""
    
    @staticmethod
    def log_alignment_status(target_x, center_x, tolerance):
        """记录对齐状态"""
        error = abs(target_x - center_x)
        if error <= tolerance:
            return f"ALIGNED (误差: {error:.1f}px)"
        else:
            direction = "左" if target_x < center_x else "右"
            return f"偏{direction} {error:.1f}px"
    
    @staticmethod
    def calculate_alignment_quality(target_x, center_x, tolerance):
        """计算对齐质量百分比"""
        error = abs(target_x - center_x)
        if error <= tolerance:
            return 100.0
        else:
            # 假设最大可接受误差为容差的5倍
            max_error = tolerance * 5
            quality = max(0, 100 * (1 - (error - tolerance) / (max_error - tolerance)))
            return quality