#!/usr/bin/env python3
"""
地图转换工具 - 用于不同格式地图之间的转换
"""

import argparse
import yaml
import numpy as np
from PIL import Image
import os

class MapConverter:
    def __init__(self):
        pass
    
    def sdf_to_map(self, sdf_file, output_dir, resolution=0.05):
        """从SDF世界生成导航地图（简化版，实际需要更复杂的解析）"""
        print(f"Converting SDF to map: {sdf_file}")
        
        # 这里只是一个示例，实际需要解析SDF文件中的障碍物信息
        # 对于RM比赛，通常会有官方地图文件
        
        # 创建示例地图（100x100的空白地图）
        width, height = 100, 100
        map_data = np.ones((height, width), dtype=np.uint8) * 255  # 全白（空闲）
        
        # 保存为PGM
        pgm_file = os.path.join(output_dir, "generated_map.pgm")
        self.save_pgm(map_data, pgm_file)
        
        # 创建YAML配置
        yaml_file = os.path.join(output_dir, "generated_map.yaml")
        yaml_data = {
            'image': 'generated_map.pgm',
            'resolution': resolution,
            'origin': [-width * resolution / 2, -height * resolution / 2, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        print(f"Map saved to: {output_dir}")
    
    def save_pgm(self, data, filename):
        """保存为PGM格式"""
        height, width = data.shape
        
        with open(filename, 'wb') as f:
            # PGM头
            f.write(f"P5\n{width} {height}\n255\n".encode())
            
            # 数据
            # 反转：0=占用，255=空闲，128=未知
            pgm_data = np.where(data == 0, 0, np.where(data == 100, 255, 128))
            f.write(pgm_data.astype(np.uint8).tobytes())
    
    def convert_image(self, input_image, output_dir, resolution=0.05):
        """转换图片为导航地图"""
        print(f"Converting image: {input_image}")
        
        img = Image.open(input_image)
        
        # 转换为灰度
        if img.mode != 'L':
            img = img.convert('L')
        
        # 转换为numpy数组
        img_array = np.array(img)
        
        # 保存为PGM
        base_name = os.path.splitext(os.path.basename(input_image))[0]
        pgm_file = os.path.join(output_dir, f"{base_name}.pgm")
        yaml_file = os.path.join(output_dir, f"{base_name}.yaml")
        
        self.save_pgm(img_array, pgm_file)
        
        # 创建YAML配置
        yaml_data = {
            'image': f"{base_name}.pgm",
            'resolution': resolution,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        print(f"Map saved to: {pgm_file}, {yaml_file}")

def main():
    parser = argparse.ArgumentParser(description='地图转换工具')
    parser.add_argument('--sdf', type=str, help='输入SDF文件')
    parser.add_argument('--image', type=str, help='输入图片文件')
    parser.add_argument('--output', type=str, default='./maps', help='输出目录')
    parser.add_argument('--resolution', type=float, default=0.05, help='地图分辨率')
    
    args = parser.parse_args()
    
    converter = MapConverter()
    
    if args.sdf:
        converter.sdf_to_map(args.sdf, args.output, args.resolution)
    elif args.image:
        converter.convert_image(args.image, args.output, args.resolution)
    else:
        print("请指定输入文件：--sdf 或 --image")

if __name__ == '__main__':
    main()