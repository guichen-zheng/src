# pb_option1_vision/object_detector_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        self.declare_parameter('model_path', 'yolo8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', ['cup', 'apple', 'banana'])
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('device', 'cpu')
        
        model_path = self.get_parameter_value('model_path')
        conf_thresh = self.get_parameter_value('confidence_threshold')
        self.target_classes = self.get_parameter_value('target_classes')
        image_topic = self.get_parameter_value('image_topic')
        
        # Handle model path: if not absolute, look in package's models directory
        if not os.path.isabs(model_path):
            package_share = get_package_share_directory('pb_option1_vision')
            candidate = os.path.join(package_share, 'models', model_path)
            if os.path.exists(candidate):
                model_path = candidate
                self.get_logger().info(f'Using package model: {model_path}')
            else:
                self.get_logger().warn(f'Package model not found, trying current path: {model_path}')
        
        self.get_logger().info(f'Initializing YOLO detector with model: {model_path}')
        try:
            self.model = YOLO(model_path)
            self.model.to(self.get_parameter_value('device'))
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            rclpy.shutdown()
            return
        
        # Map target classes to COCO IDs (partial match)
        self.class_names = self.model.names
        self.target_class_ids = {}
        for target in self.target_classes:
            for cls_id, name in self.class_names.items():
                if target in name:
                    self.target_class_ids[target] = cls_id
                    self.get_logger().info(f'Target "{target}" mapped to ID {cls_id} ({name})')
                    break
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.object_publisher = self.create_publisher(String, '/vision/detected_object', 10)
        self.position_publisher = self.create_publisher(Point, '/vision/object_position', 10)
        self.annotated_publisher = self.create_publisher(Image, '/vision/annotated_image', 10)
        
        self.bridge = CvBridge()
        
        self.get_logger().info('Object detection node started')
        self.get_logger().info(f'Subscribed to image topic: {image_topic}')

    def image_callback(self, msg):
        self.get_logger().info(f'Received image, size: {msg.width} x {msg.height}')
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {str(e)}')
            return
        
        results = self.model.predict(cv_image, conf=self.get_parameter_value('confidence_threshold'))
        self.get_logger().info(f'Detected {len(results[0].boxes)} objects')
        
        best_target = None
        best_conf = 0.0
        best_position = Point()
        
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            for target, target_id in self.target_class_ids.items():
                if cls_id == target_id and conf > best_conf:
                    best_conf = conf
                    best_target = target
                    xyxy = box.xyxy[0].cpu().numpy()
                    center_x = (xyxy[0] + xyxy[2]) / 2
                    center_y = (xyxy[1] + xyxy[3]) / 2
                    best_position.x = center_x
                    best_position.y = center_y
                    best_position.z = conf
                    break
        
        if best_target:
            obj_msg = String()
            obj_msg.data = best_target
            self.object_publisher.publish(obj_msg)
            self.position_publisher.publish(best_position)
            self.get_logger().info(f'Detected: {best_target}, confidence: {best_conf:.2f}')
        
        # Create annotated image
        self.get_logger().info('Creating annotated image')
        annotated = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        annotated_msg.header = msg.header
        self.annotated_publisher.publish(annotated_msg)
        self.get_logger().info('Published annotated image')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()