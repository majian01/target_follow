#!/bin/bash

# 摄像头跟随节点启动脚本
# 用法: ./run_cam_follow.sh [配置类型]
# 配置类型: precise, balanced, fast

echo "===================="
echo "  摄像头跟随节点"
echo "===================="

# 默认配置
CONFIG="balanced"

# 检查命令行参数
if [ $# -eq 1 ]; then
    case $1 in
        "precise"|"balanced"|"fast")
            CONFIG=$1
            ;;
        "help"|"-h"|"--help")
            echo "用法: $0 [配置类型]"
            echo ""
            echo "可用配置:"
            echo "  precise  - 精确对齐配置 (高精度，响应较慢)"
            echo "  balanced - 平衡配置 (精度和响应的平衡) [默认]"
            echo "  fast     - 快速响应配置 (响应快，精度稍低)"
            echo ""
            echo "示例:"
            echo "  $0           # 使用默认balanced配置"
            echo "  $0 precise   # 使用精确配置"
            echo "  $0 fast      # 使用快速配置"
            exit 0
            ;;
        *)
            echo "错误: 未知配置 '$1'"
            echo "可用配置: precise, balanced, fast"
            echo "使用 '$0 help' 查看详细说明"
            exit 1
            ;;
    esac
elif [ $# -gt 1 ]; then
    echo "错误: 参数过多"
    echo "使用 '$0 help' 查看使用说明"
    exit 1
fi

echo "使用配置: $CONFIG"
echo ""

# 配置说明
case $CONFIG in
    "precise")
        echo "精确对齐配置:"
        echo "  - 对齐容差: 3像素"
        echo "  - 转向系数: 0.002"
        echo "  - 最大转向速度: 0.4 rad/s"
        echo "  - 特点: 高精度对齐，响应较慢"
        ;;
    "balanced")
        echo "平衡配置:"
        echo "  - 对齐容差: 5像素"
        echo "  - 转向系数: 0.004"
        echo "  - 最大转向速度: 0.6 rad/s"
        echo "  - 特点: 精度和响应的平衡"
        ;;
    "fast")
        echo "快速响应配置:"
        echo "  - 对齐容差: 8像素"
        echo "  - 转向系数: 0.006"
        echo "  - 最大转向速度: 0.8 rad/s"
        echo "  - 特点: 快速响应，精度稍低"
        ;;
esac

echo ""
echo "启动节点..."
echo "按 Ctrl+C 停止"
echo "===================="

# 检查Python文件是否存在
if [ ! -f "cam_follow_final.py" ]; then
    echo "错误: 找不到 cam_follow_final.py 文件"
    exit 1
fi

if [ ! -f "alignment_config.py" ]; then
    echo "错误: 找不到 alignment_config.py 文件"
    exit 1
fi

# 运行节点
python cam_follow_final.py $CONFIG