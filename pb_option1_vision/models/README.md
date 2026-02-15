# models/ 目录说明

## 📁 目录用途

此目录用于存放YOLO模型文件(.pt格式)。

## 🎯 支持的模型

### 1. YOLOv8预训练模型 (COCO数据集)

| 模型文件 | 大小 | 速度 | 准确率 | 推荐用途 |
|---------|------|------|--------|---------|
| yolov8n.pt | 6 MB | 最快 | 中等 | ✅ 默认推荐 |
| yolov8s.pt | 22 MB | 快 | 良好 | 高性能设备 |
| yolov8m.pt | 52 MB | 中等 | 高 | 精度优先 |
| yolov8l.pt | 87 MB | 慢 | 很高 | 离线处理 |
| yolov8x.pt | 136 MB | 最慢 | 最高 | 专业应用 |

### 2. 自定义训练模型

如果你训练了自己的模型,请将`.pt`文件放在此目录下。

## 📥 如何获取模型

### 方式1: 自动下载(推荐)

首次运行时,系统会自动下载默认模型:

```bash
# 启动系统,会自动下载yolov8n.pt
ros2 launch pb_option1_vision vision_and_follow.launch.py
```

### 方式2: 手动下载

```bash
# 下载nano模型(推荐)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# 下载small模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt

# 移动到models目录
mv yolov8*.pt ~/ros2_ws/src/pb_option1_vision/models/
```

### 方式3: Python下载

```python
from ultralytics import YOLO

# 会自动下载到当前目录
model = YOLO('yolov8n.pt')
```

## ⚙️ 使用自定义模型

### 1. 准备模型文件

将训练好的模型文件复制到此目录:

```bash
cp /path/to/your/custom_model.pt models/
```

### 2. 修改配置文件

编辑 `config/detector_params.yaml`:

```yaml
/**:
  ros__parameters:
    model_path: "custom_model.pt"  # 改为你的模型文件名
```

### 3. 训练自定义模型(可选)

如果要训练专门识别特定物体的模型:

```python
from ultralytics import YOLO

# 加载预训练模型
model = YOLO('yolov8n.pt')

# 训练你的数据集
results = model.train(
    data='your_dataset.yaml',
    epochs=100,
    imgsz=640,
    device='0'  # GPU
)

# 保存模型
model.save('custom_model.pt')
```

## 📝 模型信息

### COCO类别ID

项目需要的三个类别在COCO数据集中的ID:

```python
41: 'cup'      # 水杯
46: 'banana'   # 香蕉
47: 'apple'    # 苹果
```

### 完整COCO类别列表

```python
0: person, 1: bicycle, 2: car, 3: motorcycle, 4: airplane,
5: bus, 6: train, 7: truck, 8: boat, 9: traffic light,
...
41: cup, 42: fork, 43: knife, 44: spoon, 45: bowl,
46: banana, 47: apple, 48: sandwich, 49: orange, 50: broccoli,
...
```

完整列表请参考: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml

## 🔍 模型选择建议

### 根据硬件选择

**CPU模式:**
```yaml
model_path: "yolov8n.pt"  # nano版本,最快
device: "cpu"
```

**GPU模式:**
```yaml
model_path: "yolov8s.pt"  # small版本,平衡性能
device: "cuda"
```

### 根据需求选择

**实时性优先(作业演示推荐):**
- 模型: yolov8n.pt
- 帧率: 12-15 FPS (CPU), 60+ FPS (GPU)
- 准确率: 85-90%

**准确率优先:**
- 模型: yolov8m.pt
- 帧率: 8-10 FPS (CPU), 40+ FPS (GPU)
- 准确率: 90-95%

## ⚠️ 注意事项

1. **文件大小**: 模型文件较大,不要上传到Git
2. **版本兼容**: 确保使用Ultralytics 8.0+版本
3. **设备选择**: GPU模式需要CUDA支持
4. **内存需求**: 至少预留1GB内存

## 📚 相关资源

- [YOLOv8官方文档](https://docs.ultralytics.com/)
- [模型训练教程](https://docs.ultralytics.com/modes/train/)
- [COCO数据集](https://cocodataset.org/)

---

**提示**: 如果模型文件不存在,系统会在首次运行时自动下载yolov8n.pt
