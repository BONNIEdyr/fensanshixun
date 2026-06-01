from maix import camera, display, image, app, time
import math

from detector import Detector
from comm import Comm
from filter import LowPassFilter
from config import *


# =========================
# 初始化
# =========================
detector = Detector(MODEL_PATH)

cam = camera.Camera(
    detector.detector.input_width(),
    detector.detector.input_height(),
    detector.detector.input_format()
)

disp = display.Display()
comm = Comm(UART_PORT, UART_BAUD)

filter_x = LowPassFilter(alpha=FILTER_ALPHA)
filter_y = LowPassFilter(alpha=FILTER_ALPHA)

fps = time.FPS()


# =========================
# 找最近圆环
# =========================
def get_best_ring(objs):
    rings = [o for o in objs if o.class_id == CLASS_RING]

    if not rings:
        return None

    def dist(o):
        cx = o.x + o.w // 2
        cy = o.y + o.h // 2
        rx = CENTER_X + REF_X
        ry = CENTER_Y - REF_Y
        return (cx - rx) ** 2 + (cy - ry) ** 2

    return min(rings, key=dist)


# =========================
# 主循环
# =========================
while not app.need_exit():

    img = cam.read()
    
    objs = detector.detect(img)
    best = get_best_ring(objs)

    if not best:

        filter_x.reset()
        filter_y.reset()

        comm.send_none()

        img.draw_string(2, 2, "NO RING", image.COLOR_RED)
        disp.show(img)
        continue
    # =========================
    # 圆心
    # =========================
    cx = best.x + best.w // 2
    cy = best.y + best.h // 2

    ref_x = CENTER_X + REF_X
    ref_y = CENTER_Y - REF_Y

    dx = cx - ref_x
    dy = ref_y - cy

    # =========================
    # 滤波
    # =========================
    dx = filter_x.update(dx)
    dy = filter_y.update(dy)

    dx = int(max(-DX_LIMIT, min(DX_LIMIT, dx)))
    dy = int(max(-DY_LIMIT, min(DY_LIMIT, dy)))

    # =========================
    # 距离
    # =========================
    dist = math.sqrt(dx * dx + dy * dy)

    # =========================
    # 到位判断
    # =========================
    if dist < STOP_DIST_THRESHOLD:
        comm.send_done()
        label = "DONE"
        color = image.COLOR_GREEN
    else:
        comm.send_ring(dx, dy)
        label = "RING"
        color = image.COLOR_BLUE

    # =========================
    # 画图
    # =========================
    img.draw_rect(best.x, best.y, best.w, best.h, color=color)
    img.draw_cross(cx, cy, color)
    img.draw_string(best.x, best.y - 18, label, color)

    img.draw_string(2, 2, f"dx:{dx} dy:{dy}", image.COLOR_WHITE)

    img.draw_cross(CENTER_X + REF_X, CENTER_Y - REF_Y, image.COLOR_YELLOW)

    img.draw_string(2, 20, f"FPS:{fps.fps():.1f}", image.COLOR_YELLOW)

    disp.show(img)