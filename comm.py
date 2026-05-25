from maix import uart
import struct

class Comm:

    def __init__(self, port="/dev/ttyS0", baud=115200):

        self.serial = uart.UART(port, baud)

    def send(self, mode, dx, dy):

        # 限幅（防止溢出 int8）
        dx = max(-127, min(127, int(dx)))
        dy = max(-127, min(127, int(dy)))

        # 协议帧：
        # 0xAA + mode + dx + dy + 0x55
        data = struct.pack(
            "<BbbbB",
            0xAA,   # 帧头
            mode,   # 模式
            dx,     # x偏差
            dy,     # y偏差
            0x55    # 帧尾
        )

        self.serial.write(data)

    def send_stop(self):
        """
        没有目标时调用
        """
        self.send(0x00, 0, 0)