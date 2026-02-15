#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class ObjectDetectorNode(Node):
    
    def __init__(self):
        super().__init__('object_detector_node')
        
        # 声明参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', ['cup', 'apple', 'banana'])
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('device', 'cpu')
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_classes = self.get_parameter('target_classes').value
        image_topic = self.get_parameter('image_topic').value
        device = self.get_parameter('device').value
        
        # 初始化YOLO模型
        self.get_logger().info('='*60)
        self.get_logger().info('初始化YOLO物体检测节点')
        self.get_logger().info('='*60)
        
        try:
            # 如果模型路径是相对路径,尝试从models目录加载
            if not os.path.isabs(model_path):
                try:
                    pkg_share = get_package_share_directory('pb_option1_vision')
                    model_full_path = os.path.join(pkg_share, 'models', model_path)
                    if os.path.exists(model_full_path):
                        model_path = model_full_path
                        self.get_logger().info(f'从包中加载模型: {model_path}')
                except:
                    pass
            
            self.get_logger().info(f'加载YOLO模型: {model_path}')
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f' YOLO模型加载成功 (设备: {device})')
            
        except Exception as e:
            self.get_logger().error(f' 模型加载失败: {str(e)}')
            raise
        
        # 获取YOLO类别名称
        self.class_names = self.model.names
        
        # 创建目标类别映射
        self.target_class_ids = {}
        for target in self.target_classes:
            for class_id, class_name in self.class_names.items():
                if target.lower() in class_name.lower():
                    self.target_class_ids[target] = class_id
                    self.get_logger().info(f'目标类别: {target} -> {class_name} (ID: {class_id})')
        
        if not self.target_class_ids:
            self.get_logger().warn(' 警告: 未找到任何目标类别!')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 当前检测到的物体
        self.current_detection = {
            'class': None,
            'confidence': 0.0,
            'center': Point()
        }
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # 发布检测到的物体类别
        self.object_pub = self.create_publisher(
            String,
            '/vision/detected_object',
            10
        )
        
        # 发布物体中心位置
        self.position_pub = self.create_publisher(
            Point,
            '/vision/object_position',
            10
        )
        
        # 发布带标注的图像
        self.annotated_pub = self.create_publisher(
            Image,
            '/vision/annotated_image',
            10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('物体检测节点初始化完成')
        self.get_logger().info(f'订阅话题: {image_topic}')
        self.get_logger().info(f'发布话题: /vision/detected_object')
        self.get_logger().info(f'发布话题: /vision/object_position')
        self.get_logger().info(f'发布话题: /vision/annotated_image')
        self.get_logger().info('='*60)
        
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO检测
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                verbose=False
            )[0]
            
            # 重置当前检测
            detected_object = None
            max_confidence = 0.0
            object_center = Point()
            
            # 处理检测结果
            for box in results.boxes:
                class_id = int(box.cls[0])
                class_name = self.class_names[class_id]
                confidence = float(box.conf[0])
                
                # 检查是否是目标类别
                is_target = False
                detected_class = None
                for target, target_id in self.target_class_ids.items():
                    if class_id == target_id:
                        is_target = True
                        detected_class = target
                        break
                
                if not is_target:
                    continue
                
                # 选择置信度最高的检测
                if confidence > max_confidence:
                    max_confidence = confidence
                    detected_object = detected_class
                    
                    # 计算物体中心
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    object_center.x = (x1 + x2) / 2.0
                    object_center.y = (y1 + y2) / 2.0
                    object_center.z = float(confidence)  # 使用z存储置信度
            
            # 发布检测结果
            if detected_object:
                # 发布物体类别
                object_msg = String()
                object_msg.data = detected_object
                self.object_pub.publish(object_msg)
                
                # 发布物体位置
                self.position_pub.publish(object_center)
                
                # 日志输出
                self.get_logger().info(
                    f'检测到: {detected_object} | '
                    f'置信度: {max_confidence:.2f} | '
                    f'位置: ({object_center.x:.0f}, {object_center.y:.0f})',
                    throttle_duration_sec=1.0
                )
            
            # 绘制标注图像
            annotated_image = results.plot()
            
            # 添加自定义文字
            if detected_object:
                cv2.putText(
                    annotated_image,
                    f'Detected: {detected_object.upper()}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2
                )
            
            # 发布标注图像
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
