import math
# 模型路径
# =========================
MODEL_PATH = "/root/models/gcmodel/model_272201.mud"


# =========================
# YOLO阈值（调参核心）
# =========================
CONF_TH = 0.5          # 置信度阈值（控制误检漏检。越高越严格）【后续调】
IOU_TH = 0.45          # NMS阈值（去重）

# =========================
# 类别定义（必须和训练一致）
# =========================
CLASS_OBJECT = 0       # 物块
CLASS_RING = 1         # 圆环


# =========================
# 图像中心（和模型输入一致）
# =========================
IMAGE_WIDTH = 224
IMAGE_HEIGHT = 224

CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2
# =========================
# 摄像头视场角（需要按实际镜头修改）
# =========================import math
FOV_X = 72.0     # 水平视场角
FOV_Y = 55.0     # 垂直视场角

focal_x = CENTER_X / math.tan(math.radians(FOV_X / 2))
focal_y = CENTER_Y / math.tan(math.radians(FOV_Y / 2))
# =========================
# UART通信配置（STM32）
# =========================
UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200


# =========================
# 控制限幅（防止溢出 int8）
# =========================
DX_LIMIT = 45
DY_LIMIT = 35


# =========================
# 抓取/对准阈值（后面状态机会用）
# =========================

# 认为“已对准”的范围【后面调参】
ALIGN_THRESHOLD_X = 3
ALIGN_THRESHOLD_Y = 3


# 可以触发“抓取”的窗口【后续调】
GRAB_THRESHOLD_X = 2
GRAB_THRESHOLD_Y = 2


# =========================
# 滤波参数，越大越稳【后续调】
# =========================
FILTER_ALPHA = 0.75

# =========================
# 通信协议定义
# =========================
PROTOCOL_HEADER = 0xAA
PROTOCOL_TAIL = 0x55

MODE_NONE = 0x00
MODE_OBJECT = 0x01
MODE_RING = 0x02


# =========================
# 状态机预留，后面会用
# =========================
STATE_SEARCH = 0
STATE_ALIGN = 1
STATE_GRAB = 2
STATE_LIFT = 3
STATE_FIND_RING = 4
STATE_PLACE = 5