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
# 目标锁定
# =========================
locked = False
locked_target = None

# =========================
# 防抖状态
# =========================
stable_count = 0
STABLE_FRAME = 5   # 连续稳定帧数才算DONE


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

    # =========================
    # 未锁定 → 选目标
    # =========================
    if not locked:
        target = get_best_ring(objs)

        if target:
            locked_target = target
            locked = True
            stable_count = 0

    # =========================
    # 已锁定 → 固定目标
    # =========================
    else:
        target = locked_target

        # 丢失目标 → 解锁
        if target is None:
            locked = False
            filter_x.reset()
            filter_y.reset()
            comm.send_mode(0x00, 0, 0)

            img.draw_string(2, 2, "LOST", image.COLOR_RED)
            disp.show(img)
            continue

    # =========================
    # 计算中心
    # =========================
    cx = target.x + target.w // 2
    cy = target.y + target.h // 2

    ref_x = CENTER_X + REF_X
    ref_y = CENTER_Y - REF_Y

    dx = cx - ref_x
    dy = ref_y - cy

    dx = filter_x.update(dx)
    dy = filter_y.update(dy)

    dx = int(max(-DX_LIMIT, min(DX_LIMIT, dx)))
    dy = int(max(-DY_LIMIT, min(DY_LIMIT, dy)))

    dist = math.sqrt(dx * dx + dy * dy)

    # =========================
    # 状态判断（核心）
    # =========================

    # ① 接近目标 → 计数稳定
    if dist < STOP_DIST_THRESHOLD:
        stable_count += 1
    else:
        stable_count = 0

    # =========================
    # 通信输出（三状态）
    # =========================

    if not locked:
        mode = 0x00      # LOST
        label = "LOST"
        color = image.COLOR_RED

    elif stable_count >= STABLE_FRAME:
        mode = 0x02      # DONE
        label = "DONE"
        color = image.COLOR_GREEN

        # DONE后自动解锁（进入下一阶段）
        locked = False
        locked_target = None

    else:
        mode = 0x01      # TRACK
        label = "TRACK"
        color = image.COLOR_BLUE

    comm.send(mode, dx, dy)

    # =========================
    # 可视化
    # =========================
    img.draw_rect(target.x, target.y, target.w, target.h, color=color)
    img.draw_cross(cx, cy, color)

    img.draw_string(target.x, target.y - 18, label, color)
    img.draw_string(2, 2, f"dx:{dx} dy:{dy}", image.COLOR_WHITE)

    img.draw_cross(ref_x, ref_y, image.COLOR_YELLOW)

    img.draw_string(
        2, 20,
        f"lock:{locked} stable:{stable_count} fps:{fps.fps():.1f}",
        image.COLOR_YELLOW
    )

    disp.show(img)