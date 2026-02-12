from setuptools import setup
import os
from glob import glob

package_name = 'pb_option1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键：把 launch 文件夹下的所有 .py 文件安装到 share/pb_option1_bringup/launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='your@email.com',
    description='Bringup for pb_option1 simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 如果以后有 Python 节点，可以在这里加
            # 'some_node = pb_option1_bringup.some_node:main',
        ],
    },
)