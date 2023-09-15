from ultralytics import YOLO
import time

class Position:
    def __init__(self):
        self.center_point = [0, 0]
        self.vertices = [[0, 0], [0, 0], [0, 0], [0, 0]]

def detect():
    # TODO: xxxxx
    time_start = time.time()
    # 加载模型
    model = YOLO("/home/jalen/code/RoboMaster/visual/Assessment/complex_task/best.pt")  # 加载预训练模型（建议用于训练）

    result = model("/home/jalen/code/RoboMaster/visual/Assessment/complex_task/cola/train/images/01_jpg.rf.54210228de63d663b8a729922c71adf2.jpg")  # 对图像进行预测

    print(result.numpy())

    position = Position()
    # position.center_point
    # position.vertices

    time_end = time.time()
    return position, time_end - time_start

if __name__ == '__main__':
    detect()