import onnxruntime as ort
session = ort.InferenceSession("yolov8n.onnx")
print("ONNX 模型加载成功")