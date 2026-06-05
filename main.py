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
# 跟踪状态
# =========================

locked = False

lock_cx = 0
lock_cy = 0

stable_count = 0
lost_count = 0

STABLE_FRAME = 5
LOST_FRAME = 5


# =========================
# 找离参考点最近的圆环
# =========================

def get_best_ring(objs):

    rings = [
        o for o in objs
        if o.class_id == CLASS_RING
    ]

    if not rings:
        return None

    rx = CENTER_X + REF_X
    ry = CENTER_Y - REF_Y

    def dist(o):

        cx = o.x + o.w // 2
        cy = o.y + o.h // 2

        return (
            (cx - rx) ** 2 +
            (cy - ry) ** 2
        )

    return min(rings, key=dist)


# =========================
# 锁定后跟踪最近目标
# =========================

def find_nearest_ring(objs, last_cx, last_cy):

    rings = [
        o for o in objs
        if o.class_id == CLASS_RING
    ]

    if not rings:
        return None

    def dist(o):

        cx = o.x + o.w // 2
        cy = o.y + o.h // 2

        return (
            (cx - last_cx) ** 2 +
            (cy - last_cy) ** 2
        )

    return min(rings, key=dist)


# =========================
# 主循环
# =========================

while not app.need_exit():

    img = cam.read()

    objs = detector.detect(img)

    # =========================
    # 未锁定
    # =========================

    if not locked:

        target = get_best_ring(objs)

        if target:

            lock_cx = target.x + target.w // 2
            lock_cy = target.y + target.h // 2

            locked = True

            stable_count = 0
            lost_count = 0

        else:

            filter_x.reset()
            filter_y.reset()

            comm.send_lost()

            img.draw_string(
                2,
                2,
                "NO RING",
                image.COLOR_RED
            )

            img.draw_string(
                2,
                20,
                f"FPS:{fps.fps():.1f}",
                image.COLOR_YELLOW
            )

            disp.show(img)

            continue

    # =========================
    # 已锁定
    # =========================

    else:

        target = find_nearest_ring(
            objs,
            lock_cx,
            lock_cy
        )

        if target:

            lost_count = 0

            lock_cx = target.x + target.w // 2
            lock_cy = target.y + target.h // 2

        else:

            lost_count += 1

            if lost_count >= LOST_FRAME:

                locked = False

                filter_x.reset()
                filter_y.reset()

                comm.send_lost()

                img.draw_string(
                    2,
                    2,
                    "LOST",
                    image.COLOR_RED
                )

                disp.show(img)

                continue

            else:

                disp.show(img)

                continue

    # =========================
    # 计算偏差
    # =========================

    cx = lock_cx
    cy = lock_cy

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
    # DONE判断
    # =========================

    if dist < STOP_DIST_THRESHOLD:

        stable_count += 1

    else:

        stable_count = 0

    # =========================
    # 通信
    # =========================

    if stable_count >= STABLE_FRAME:

        comm.send_done()

        label = "DONE"
        color = image.COLOR_GREEN

        locked = False

    else:

        comm.send_track(dx, dy)

        label = "TRACK"
        color = image.COLOR_BLUE

    # =========================
    # 可视化
    # =========================

    img.draw_rect(
        target.x,
        target.y,
        target.w,
        target.h,
        color=color
    )

    img.draw_cross(
        cx,
        cy,
        color
    )

    img.draw_cross(
        ref_x,
        ref_y,
        image.COLOR_YELLOW
    )

    img.draw_string(
        target.x,
        target.y - 18,
        label,
        color
    )

    img.draw_string(
        2,
        2,
        f"dx:{dx} dy:{dy}",
        image.COLOR_WHITE
    )

    img.draw_string(
        2,
        20,
        f"lock:{locked} stable:{stable_count} fps:{fps.fps():.1f}",
        image.COLOR_YELLOW
    )

    disp.show(img)