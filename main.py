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
        # 参考点坐标（图像坐标系）
        # =========================
        ref_x = CENTER_X + REF_X
        ref_y = CENTER_Y - REF_Y

        # =========================
        # 相对参考点的偏差
        # =========================
        dx = cx - ref_x
        dy = ref_y - cy

        # =========================
        # 限幅（防止超int8）
        # =========================
        dx = max(-DX_LIMIT, min(DX_LIMIT, dx))
        dy = max(-DY_LIMIT, min(DY_LIMIT, dy))

        # =========================
        # 低通滤波
        # =========================
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
        # 偏差模长
        # =========================
        dist = math.sqrt(dx * dx + dy * dy)

        # =========================
        # 发送串口
        # 对准时（模长小于阈值）发送停止指令
        # =========================
        if dist < STOP_DIST_THRESHOLD:
            comm.send_stop()
        else:
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

    # =========================
    # 参考点（REF_X, REF_Y 偏移后的位置）
    # =========================
    ref_draw_x = CENTER_X + REF_X
    ref_draw_y = CENTER_Y - REF_Y
    img.draw_circle(
        ref_draw_x,
        ref_draw_y,
        3,
        image.COLOR_YELLOW
    )
    img.draw_string(
        ref_draw_x + 5,
        ref_draw_y - 10,
        f"REF({REF_X},{REF_Y})",
        image.COLOR_YELLOW
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