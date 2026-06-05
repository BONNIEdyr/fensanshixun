from maix import nn
from config import CONF_TH, IOU_TH

class Detector:
    """
    YOLO检测器封装
    当前版本只用于圆环检测（ring placement）负责“检测 + 返回结果”
    """

    def __init__(self, model_path):

        self.detector = nn.YOLOv5(model=model_path)

        self.input_width = self.detector.input_width()
        self.input_height = self.detector.input_height()
        self.input_format = self.detector.input_format()

        self.labels = self.detector.labels

        print("========== YOLO MODEL INFO ==========")
        print("labels:", self.labels)
        print("input size:", self.input_width, "x", self.input_height)
        print("=====================================")

    # =========================
    # 执行检测
    # =========================
    def detect(self, img):

        objs = self.detector.detect(
            img,
            conf_th=CONF_TH,
            iou_th=IOU_TH
        )

        return objs

    # =========================
    # 获取圆环目标（基础过滤）
    # 只做“类别筛选”，不做排序策略
    # =========================
    def get_rings(self, objs):

        if not objs:
            return []

        return [
            o for o in objs
            if o.class_id == 1   # CLASS_RING
        ]

    # =========================
    # 获取所有目标（备用）
    # =========================
    def get_all(self, objs):
        return objs