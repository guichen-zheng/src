from ultralytics import YOLO
import torch

# 加载模型（使用你下载的yolov8n.pt）
model = YOLO('yolov8n.pt')

# 导出为ONNX，关键参数设置
model.export(
    format='onnx',
    imgsz=640,          # 固定输入尺寸
    opset=11,           # OpenCV 4.5.4 支持的最高opset
    simplify=True,      # 必须！使用onnxsim简化模型
    dynamic=False,      # 禁用动态轴，保持静态图
    half=False          # 不使用半精度，保证兼容性
)
print("ONNX模型导出完成并已简化。")