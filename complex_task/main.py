from ultralytics import YOLO

# 加载模型
model = YOLO("yolov8n.pt")  # 加载预训练模型（建议用于训练）

# 使用模型
model.train(data="/home/jalen/code/RoboMaster/visual/Assessment/complex_task/cola/data.yaml", epochs=3)  # 训练模型
metrics = model.val()  # 在验证集上评估模型性能
results = model("/home/jalen/code/RoboMaster/visual/Assessment/complex_task/cola/train/images/01_jpg.rf.54210228de63d663b8a729922c71adf2.jpg")  # 对图像进行预测
success = model.export(format="onnx")  # 将模型导出为 ONNX 格式
