# the_final_exam
## 总的框架如下
```
src/
|——my_robot_description                    #机器人描述文件
|——my_robot_bringup                        #实现对robot、world的调用，导航、建图实现
|——my_decetor                              #实现物品识别
|——deceted_cmd                             #接收decetor，然后调用nav2实现相应动作
```
目前结构如此，后续可作出相应更改
### 一、my_robot_description
使用仓库中已有模型，这里不多赘述 ...
### 二、my_robot_bringup
考虑到分仿真和现实调用，决定使用两类launch主文件，分别实现仿真和现实，还有一个launch文件打开slam建图。
文件结构如下：
```
my_robot_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/                    # Python启动文件
│   ├── simulation/
│   │   ├── simulation.launch.py      # 仿真主启动文件
│   │   ├── simulation_nav.launch.py # 仿真导航启动
│   │   └── simulation_slam.launch.py # 仿真SLAM启动
│   ├── real_robot/
│   │   ├── real_robot.launch.py     # 实物主启动文件
│   │   ├── real_nav.launch.py       # 实物导航启动
│   │   └── real_slam.launch.py      # 实物SLAM启动
│   └── common/
│       └── navigation_core.launch.py # 核心导航启动
├── config/
│   ├── simulation/
│   │   ├── nav2_params.yaml         # 仿真导航参数
│   │   └── slam_params.yaml         # 仿真SLAM参数
│   └── real_robot/
│       ├── nav2_params.yaml         # 实物导航参数
│       └── slam_params.yaml         # 实物SLAM参数
├── worlds/                    # Gazebo世界文件
│   └── 
├── maps/                      # 地图文件
│   ├── simulation/
│   │   └── (仿真的地图文件.yaml和.pgm)
│   └── real_robot/
│       └── (实物的地图文件.yaml和.pgm)
├── include/                  # C++头文件
│   └── my_robot_bringup/
│       ├── navigation_core.hpp
│       ├── slam_processor.hpp
│       └── map_manager.hpp
├── src/                      # C++源文件
│   ├── navigation_core.cpp   # 导航核心功能
│   ├── slam_processor.cpp    # SLAM处理
│   ├── map_manager.cpp       # 地图管理
│   └── main.cpp              # 主程序
└── test/                     # 测试文件
    └── test_navigation.cpp
```
目前结构如此，后续可作出相应更改