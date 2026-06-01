from maix import uart
import struct

class Comm:
    """
    摄像头 → STM32

    协议：
    0xAA + mode + dx + dy + 0x55

    mode:
        0x00 → 无目标 / 已完成对准（STOP）
        0x02 → 圆环对准中
    """

    def __init__(self, port="/dev/ttyS0", baud=115200):
        self.serial = uart.UART(port, baud)

    def send(self, mode, dx, dy):
        """
        通用发送
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
    # 无目标 / 已完成
    # =========================
    def send_none(self):
        self.send(0x00, 0, 0)

    def send_done(self):
        self.send(0x00, 0, 0)

    # =========================
    # 圆环对准
    # =========================
    def send_ring(self, dx, dy):
        self.send(0x02, dx, dy)