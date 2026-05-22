from maix import nn
from config import *

class Detector:

    def __init__(self, model_path):

        # 加载 YOLO 模型
        self.detector = nn.YOLOv5(
            model=model_path
        )

        # 模型输入参数
        self.input_width = self.detector.input_width()
        self.input_height = self.detector.input_height()
        self.input_format = self.detector.input_format()

        # 标签
        self.labels = self.detector.labels

        print("========== YOLO MODEL INFO ==========")
        print("labels:", self.labels)
        print("input size:", self.input_width, "x", self.input_height)
        print("=====================================")

    # =========================
    # 检测函数
    # 输入：img
    # 输出：objs
    # =========================
    def detect(self, img):

        objs = self.detector.detect(
            img,
            conf_th=CONF_TH,
            iou_th=IOU_TH
        )

        return objs

    # =========================
    # 获取最佳目标
    # 默认：
    # 选择面积最大的目标
    # =========================
    def get_best_target(self, objs):

        if not objs:
            return None

        best = max(
            objs,
            key=lambda o: o.w * o.h
        )

        return best