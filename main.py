from maix import camera, display, image, app, time

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
fps = time.fps()

# =========================
# 获取最佳目标
# 当前策略：
# 选择面积最大的目标
# =========================
def get_best_target(objs):

    if not objs:
        return None

    # 后续可改：
    # 1. 最大面积
    # 2. 最近中心
    # 3. 最高置信度

    best = max(objs, key=lambda o: o.w * o.h)

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

        # 目标中心
        cx = best.x + best.w // 2
        cy = best.y + best.h // 2

        # 相对画面中心偏差
        dx = cx - CENTER_X
        dy = cy - CENTER_Y

        # 限幅（防止超int8）
        dx = max(-DX_LIMIT, min(DX_LIMIT, dx))
        dy = max(-DY_LIMIT, min(DY_LIMIT, dy))

        # 滤波
        dx = int(filter_x.update(dx))
        dy = int(filter_y.update(dy))

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
        comm.send(mode, dx, dy)

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

        # 偏差显示
        img.draw_string(
            2,
            2,
            f"dx:{dx} dy:{dy}",
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

    # FPS显示
    img.draw_string(
        2,
        20,
        f"FPS:{fps.fps():.1f}",
        image.COLOR_YELLOW
    )

    # 显示图像
    disp.show(img)