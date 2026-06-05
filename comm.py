from maix import uart
import struct


class Comm:
    """
    摄像头 → STM32 通信协议

    帧格式：
    0xAA + mode + dx + dy + 0x55

    mode 定义：
        0x00 → LOST（无目标）
        0x01 → TRACK（对准中）
        0x02 → DONE（已对准）
        0x03 → PLACE_READY（允许放置）
    """

    def __init__(self, port="/dev/ttyS0", baud=115200):
        self.serial = uart.UART(port, baud)

    def send(self, mode, dx, dy):
        """
        通用发送接口
        """

        dx = int(max(-127, min(127, dx)))
        dy = int(max(-127, min(127, dy)))

        data = struct.pack(
            "<BbbbB",
            0xAA,
            mode,
            dx,
            dy,
            0x55
        )

        self.serial.write(data)

    # =========================
    # 1. 无目标
    # =========================
    def send_lost(self):
        self.send(0x00, 0, 0)

    # =========================
    # 2. 对准中
    # =========================
    def send_track(self, dx, dy):
        self.send(0x01, dx, dy)

    # =========================
    # 3. 已对准
    # =========================
    def send_done(self):
        self.send(0x02, 0, 0)

    # =========================
    # 4. 可执行放置
    # =========================
    def send_place_ready(self):
        self.send(0x03, 0, 0)