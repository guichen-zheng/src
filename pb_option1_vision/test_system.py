#!/usr/bin/env python3
"""
系统测试脚本
检查所有依赖和配置是否正确
"""

import sys
import subprocess

def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}")

def check_python_package(package_name):
    """检查Python包"""
    try:
        __import__(package_name)
        return True
    except ImportError:
        return False

def check_ros_package(package_name):
    """检查ROS 2包"""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', package_name],
            capture_output=True,
            check=True
        )
        return True
    except subprocess.CalledProcessError:
        return False

def main():
    print_header("pb_option1_vision 系统测试")
    
    all_ok = True
    
    # 检查Python依赖
    print("\n[1] Python依赖检查")
    required_packages = {
        'rclpy': 'ROS 2 Python客户端库',
        'cv2': 'OpenCV',
        'ultralytics': 'YOLOv8',
        'numpy': 'NumPy'
    }
    
    for package, name in required_packages.items():
        if check_python_package(package):
            print(f"   {name}")
        else:
            print(f"   {name} - 请安装: pip install {package}")
            all_ok = False
    
    # 检查ROS 2包
    print("\n[2] ROS 2包检查")
    ros_packages = [
        'cv_bridge',
        'vision_msgs',
        'image_transport'
    ]
    
    for pkg in ros_packages:
        if check_ros_package(pkg):
            print(f"   {pkg}")
        else:
            print(f"   {pkg} - 请安装: sudo apt install ros-$ROS_DISTRO-{pkg.replace('_', '-')}")
            all_ok = False
    
    # 测试YOLO模型
    print("\n[3] YOLO模型测试")
    try:
        from ultralytics import YOLO
        print("   加载YOLOv8模型...")
        model = YOLO('yolov8n.pt')
        print("   YOLO模型加载成功")
        print(f"     支持类别数: {len(model.names)}")
        
        # 检查目标类别
        target_classes = ['cup', 'apple', 'banana']
        print("\n  检查目标类别:")
        for target in target_classes:
            found = False
            for class_id, class_name in model.names.items():
                if target in class_name.lower():
                    print(f"     {target} -> {class_name} (ID: {class_id})")
                    found = True
                    break
            if not found:
                print(f"     {target} 未找到")
                all_ok = False
                
    except Exception as e:
        print(f"   YOLO模型测试失败: {str(e)}")
        all_ok = False
    
    # 检查相机设备
    print("\n[4] 相机设备检查")
    import os
    video_devices = [f for f in os.listdir('/dev') if f.startswith('video')]
    if video_devices:
        print(f"   找到相机设备:")
        for dev in sorted(video_devices):
            print(f"     /dev/{dev}")
    else:
        print("    未找到相机设备")
        print("     请连接USB相机或使用仿真环境")
    
    # 检查GPU
    print("\n[5] GPU支持检查")
    try:
        import torch
        if torch.cuda.is_available():
            print("   检测到GPU")
            print(f"     GPU数量: {torch.cuda.device_count()}")
            print(f"     CUDA版本: {torch.version.cuda}")
        else:
            print("    未检测到GPU,将使用CPU模式")
    except:
        print("    未安装PyTorch,将使用CPU模式")
    
    # 总结
    print_header("测试结果总结")
    if all_ok:
        print("   所有检查通过!")
        print("\n  可以开始使用系统:")
        print("    ros2 launch pb_option1_vision vision_and_follow.launch.py")
    else:
        print("   部分检查失败")
        print("  请根据上述提示安装缺失的组件")
    
    print("\n" + "="*60 + "\n")
    return 0 if all_ok else 1

if __name__ == '__main__':
    sys.exit(main())
