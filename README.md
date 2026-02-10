# the_final_exam
## 总的框架如下
文件结构如下：
```
pb_option1_nav_vision/
├── pb_option1_bringup/               # 启动文件集合
│   ├── launch/
│   │   ├── sim.launch.py             # 仿真启动
│   │   ├── real.launch.py            # 实车启动
│   │   └── joystick.launch.py        # 可选：手柄遥控调试用
│
├── pb_option1_description/           # URDF / xacro 机器人模型（基本保持原样）
│
├── pb_option1_navigation/            # 核心导航部分（从 pb2025_sentry_nav 精简而来）
│   ├── config/
│   │   ├── nav2_params.yaml          # nav2 参数（控制器、规划器、BT等）
│   │   ├── slam_params.yaml          # slam 参数（通常用 cartographer 或 slam_toolbox）
│   │   └── amcl_params.yaml          # 定位参数（仿真和实车可能不同）
│   ├── launch/
│   │   ├── slam.launch.py
│   │   ├── localization.launch.py
│   │   └── nav2_bringup.launch.py
│   └── maps/                         # 保存的地图文件
│
├── pb_option1_vision/                # 新增：视觉识别与物体跟随
│   ├── src/
│   │   ├── object_detector_node.cpp / object_detector.py     # YOLO / yolov8 / MobileNet-SSD 等
│   │   ├── follow_behavior_node.py                           # 跟随逻辑
│   │   └── command_interpreter_node.py                       # 物体 → 动作映射
│   ├── config/
│   │   ├── detector_params.yaml      # 模型路径、置信度、类别等
│   │   └── follow_params.yaml        # 跟随距离、速度、PID 等
│   ├── launch/
│   │   └── vision_and_follow.launch.py
│   └── models/                       # 存放训练好的 .pt / .onnx 模型（或预训练模型）
│
├── pb_option1_sim/                   # 仿真专用（基于 pb_rm_simulation 精简）
│   ├── worlds/                       # 自定义仿真场景（放水杯、苹果、香蕉模型等）
│   └── launch/
│       └── gazebo_with_objects.launch.py
└── README.md
```
目前结构如此，后续可作出相应更改
### 一、pb_option1_bringup
存放launch启动文件
### 二、pb_option1_description
仓库中已有模型，此处不再赘述
