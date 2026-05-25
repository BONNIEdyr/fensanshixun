from maix import camera, display, image, app, time
import math

from detector import Detector
from comm import Comm
from filter import LowPassFilter
from config import *


# =========================
# 初始化
# =========================

# YOLO检测器
detector = Detector(MODEL_PATH)

# 摄像头
cam = camera.Camera(
    detector.detector.input_width(),
    detector.detector.input_height(),
    detector.detector.input_format()
)

# 显示
disp = display.Display()

# 串口
comm = Comm(UART_PORT, UART_BAUD)

# 低通滤波
filter_x = LowPassFilter(alpha=FILTER_ALPHA)
filter_y = LowPassFilter(alpha=FILTER_ALPHA)

# FPS
fps = time.FPS()


# =========================
# 获取最佳目标
# 当前策略：
# 选择面积最大的目标
# =========================
def get_best_target(objs):

    if not objs:
        return None

    best = max(
        objs,
        key=lambda o: o.w * o.h
    )

    return best


# =========================
# 主循环
# =========================
while not app.need_exit():

    # 读取图像
    img = cam.read()

    # YOLO检测
    objs = detector.detect(img)

    # 获取最佳目标
    best = get_best_target(objs)

    # =========================
    # 检测到目标
    # =========================
    if best:

        # =========================
        # 目标中心
        # =========================
        cx = best.x + best.w // 2
        cy = best.y + best.h // 2

        # =========================
        # 像素偏差
        # =========================
        dx = cx - CENTER_X
        dy = cy - CENTER_Y

        # =========================
        # 像素 -> 角度
        # atan版本（更准确）
        # =========================
        angle_x = math.degrees(
            math.atan(dx / focal_x)
        )

        angle_y = math.degrees(
            math.atan(dy / focal_y)
        )

        # =========================
        # 低通滤波（滤角度）
        # =========================
        angle_x = filter_x.update(angle_x)
        angle_y = filter_y.update(angle_y)

        # =========================
        # 限幅
        # =========================
        angle_x = max(-45, min(45, angle_x))
        angle_y = max(-35, min(35, angle_y))

        # =========================
        # 转int（串口发送需要）
        # =========================
        angle_x = int(angle_x)
        angle_y = int(angle_y)

        # =========================
        # 类别判断
        # =========================
        if best.class_id == CLASS_OBJECT:

            mode = MODE_OBJECT
            color = image.COLOR_GREEN
            label_name = "OBJECT"

        elif best.class_id == CLASS_RING:

            mode = MODE_RING
            color = image.COLOR_BLUE
            label_name = "RING"

        else:

            mode = MODE_NONE
            color = image.COLOR_RED
            label_name = "UNKNOWN"

        # =========================
        # 发送串口
        # =========================
        comm.send(mode, angle_x, angle_y)

        # =========================
        # 调试显示
        # =========================

        # 目标框
        img.draw_rect(
            best.x,
            best.y,
            best.w,
            best.h,
            color=color
        )

        # 中心点
        img.draw_cross(cx, cy, color)

        # 标签
        img.draw_string(
            best.x,
            best.y - 18,
            f"{label_name} {best.score:.2f}",
            color
        )

        # 角度显示
        img.draw_string(
            2,
            2,
            f"ax:{angle_x} ay:{angle_y}",
            image.COLOR_WHITE
        )

    # =========================
    # 未检测到目标
    # =========================
    else:

        comm.send(MODE_NONE, 0, 0)

        img.draw_string(
            2,
            2,
            "NO TARGET",
            image.COLOR_RED
        )

    # =========================
    # 画面中心
    # =========================
    img.draw_cross(
        CENTER_X,
        CENTER_Y,
        image.COLOR_WHITE
    )

    # =========================
    # FPS显示
    # =========================
    img.draw_string(
        2,
        20,
        f"FPS:{fps.fps():.1f}",
        image.COLOR_YELLOW
    )

    # 显示图像
    disp.show(img)