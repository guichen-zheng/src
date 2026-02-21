# setup.py

from setuptools import setup

package_name = 'pb_option1_vision'

setup(
    name=package_name,
    version='0.0.1',  # 根据原仓库版本调整
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/*.launch.py']),  # 假设有 launch 文件
        ('share/' + package_name + '/config', ['config/detector_params.yaml']),  # config 文件
        ('share/' + package_name + '/models', ['models/yolo8n.pt']),  # 模型文件，如果放在这里
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guichen-zheng',  # 根据原仓库调整
    maintainer_email='your_email@example.com',
    description='Vision package for pb_option1 using YOLOv8',
    license='Apache-2.0',  # 根据原仓库调整
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = pb_option1_vision.object_detector_node:main',
            'follow_behavior_node = pb_option1_vision.follow_behavior_node:main',
            'command_interpreter_node = pb_option1_vision.command_interpreter_node:main',
        ],
    },
)