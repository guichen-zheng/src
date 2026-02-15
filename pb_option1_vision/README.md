# pb_option1_vision - 基于YOLO的物体识别与跟随系统

**大作业选项一: 建图+导航+仿真+基础物体识别**

## 📋 项目概述

本项目实现了基于YOLOv8的物体识别与机器人控制系统,满足大作业选项一的要求:

### 核心功能

1. **物体识别** - 识别环境中的水杯、苹果、香蕉
2. **动作映射** - 根据识别结果控制机器人:
   - 🥤 **水杯** → **前行**
   - 🍎 **苹果** → **左转**
   - 🍌 **香蕉** → **右转**
3. **跟随模式** - 可选的物体跟随功能

## 📁 项目结构

```
pb_option1_vision/
├── src/                                # 源代码目录
│   ├── object_detector.py              # YOLO物体检测节点
│   ├── command_interpreter_node.py     # 命令解释节点(物体→动作)
│   └── follow_behavior_node.py         # 跟随行为节点(可选)
│
├── config/                             # 配置文件
│   ├── detector_params.yaml            # YOLO检测参数
│   └── follow_params.yaml              # 跟随行为参数
│
├── launch/                             # 启动文件
│   └── vision_and_follow.launch.py     # 主启动文件
│
├── models/                             # YOLO模型文件
│   └── (放置自定义模型.pt文件)
│
├── package.xml                         # ROS 2包描述
├── CMakeLists.txt                      # 构建配置
└── README.md                           # 本文件
```

## 🚀 快速开始

### 1. 安装依赖

```bash
# Python依赖
pip install ultralytics opencv-python

# ROS 2依赖
sudo apt install \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-msgs
```

### 2. 编译包

```bash
# 进入工作空间
cd ~/ros2_ws

# 编译
colcon build --packages-select pb_option1_vision

# Source环境
source install/setup.bash
```

### 3. 运行系统

#### 方式A: 物体识别控制模式 (演示作业要求)

```bash
# 终端1: 启动相机
ros2 run image_tools cam2image --ros-args -p device_id:=0

# 终端2: 启动视觉系统(命令模式)
ros2 launch pb_option1_vision vision_and_follow.launch.py mode:=command

# 系统会根据检测到的物体自动执行对应动作:
# - 水杯 -> 前行
# - 苹果 -> 左转  
# - 香蕉 -> 右转
```

#### 方式B: 跟随模式 (演示跟随功能)

```bash
# 终端1: 启动相机
ros2 run image_tools cam2image --ros-args -p device_id:=0

# 终端2: 启动视觉系统(跟随模式)
ros2 launch pb_option1_vision vision_and_follow.launch.py mode:=follow

# 机器人会跟随检测到的水杯移动
```

#### 方式C: 在仿真中运行

```bash
# 终端1: 启动仿真环境
ros2 launch pb_option1_bringup sim.launch.py

# 终端2: 启动视觉系统
ros2 launch pb_option1_vision vision_and_follow.launch.py \
    mode:=command \
    use_sim_time:=true
```

## ⚙️ 配置说明

### 检测参数 (`config/detector_params.yaml`)

```yaml
# YOLO模型
model_path: "yolov8n.pt"          # 模型文件

# 检测参数
confidence_threshold: 0.5         # 置信度阈值

# 目标类别
target_classes:
  - "cup"      # 水杯
  - "apple"    # 苹果
  - "banana"   # 香蕉

# 设备
device: "cpu"                     # "cpu" 或 "cuda"
```

### 跟随参数 (`config/follow_params.yaml`)

```yaml
# 跟随模式开关
enable_follow: false              # 命令模式下设为false

# 运动参数
linear_speed: 0.3                 # 线速度 (m/s)
angular_speed: 0.5                # 角速度 (rad/s)

# 目标物体
target_object: "cup"              # 跟随的物体类型
```

## 📊 话题说明

### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/vision/detected_object` | `std_msgs/String` | 检测到的物体类型 |
| `/vision/object_position` | `geometry_msgs/Point` | 物体中心位置 |
| `/vision/annotated_image` | `sensor_msgs/Image` | 带标注的图像 |
| `/vision/current_action` | `std_msgs/String` | 当前执行的动作 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 |

### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 相机图像输入 |

## 🎯 功能演示

### 1. 物体识别 → 动作控制

```
检测到水杯 -> 机器人前行
检测到苹果 -> 机器人左转
检测到香蕉 -> 机器人右转
```

### 2. 查看检测结果

```bash
# 查看检测到的物体
ros2 topic echo /vision/detected_object

# 查看当前动作
ros2 topic echo /vision/current_action

# 查看速度命令
ros2 topic echo /cmd_vel
```

### 3. 可视化

```bash
# 查看带标注的图像
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

## 🔧 调试与优化

### 提高检测准确率

```yaml
# config/detector_params.yaml
confidence_threshold: 0.7         # 提高阈值,减少误检
```

### 启用GPU加速

```yaml
# config/detector_params.yaml
device: "cuda"                    # 或 "0" 使用第一个GPU
```

### 调整运动参数

```yaml
# config/follow_params.yaml
linear_speed: 0.2                 # 降低速度更安全
angular_speed: 0.3
```

## 📝 开发文档

### 节点说明

#### 1. object_detector.py
- **功能**: YOLO物体检测
- **输入**: 相机图像
- **输出**: 检测结果(类别和位置)

#### 2. command_interpreter_node.py
- **功能**: 物体到动作的映射
- **输入**: 检测到的物体类型
- **输出**: 机器人速度命令
- **映射规则**:
  ```python
  'cup': 前行 (linear_x=0.3, angular_z=0)
  'apple': 左转 (linear_x=0, angular_z=0.5)
  'banana': 右转 (linear_x=0, angular_z=-0.5)
  ```

#### 3. follow_behavior_node.py
- **功能**: 物体跟随(可选)
- **输入**: 物体类型和位置
- **输出**: 跟随控制命令

### 系统架构

```
相机 -> object_detector -> command_interpreter -> /cmd_vel -> 机器人
                    ↓
              annotated_image
```

## ⚠️ 常见问题

### Q1: 检测不到物体?

**解决方案:**
1. 降低置信度阈值: `confidence_threshold: 0.3`
2. 确保光照充足
3. 物体要在相机视野内且清晰可见

### Q2: 机器人不动?

**解决方案:**
1. 检查是否检测到物体: `ros2 topic echo /vision/detected_object`
2. 检查速度命令: `ros2 topic echo /cmd_vel`
3. 确认机器人底盘正常工作

### Q3: 检测速度慢?

**解决方案:**
1. 使用GPU: `device: "cuda"`
2. 使用更小的模型: `model_path: "yolov8n.pt"`

### Q4: 想使用自定义模型?

**解决方案:**
1. 将训练好的模型放在 `models/` 目录
2. 修改配置: `model_path: "custom_model.pt"`

## 📚 参考资料

- [YOLOv8文档](https://docs.ultralytics.com/)
- [ROS 2文档](https://docs.ros.org/)
- [cv_bridge教程](http://wiki.ros.org/cv_bridge/Tutorials)

## 👥 作业说明

### 实现的功能点

- ✅ 物体识别(水杯、苹果、香蕉)
- ✅ 物体识别 → 动作映射(前行、左转、右转)
- ✅ 跟随物体移动功能
- ✅ 实时可视化
- ✅ 完整的开发文档

### 测试场景

1. **场景1**: 放置水杯 → 机器人前行
2. **场景2**: 放置苹果 → 机器人左转
3. **场景3**: 放置香蕉 → 机器人右转
4. **场景4**: 跟随模式 → 机器人跟随水杯移动

### 注意事项

1. 确保物体在相机视野内
2. 保持良好的光照条件
3. 物体要足够大且清晰
4. 背景尽量简单,避免干扰

## 📄 许可证

MIT License

---

**开发者**: [你的名字]  
**日期**: 2024年2月  
**课程**: ROS 2机器人开发  
**作业**: 大作业选项一
