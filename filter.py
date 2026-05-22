class LowPassFilter:

    def __init__(self, alpha=0.6):
        """
        alpha 越大 → 越稳定，但反应慢
        alpha 越小 → 反应快，但抖动大
        """
        self.alpha = alpha
        self.value = None

    def reset(self):
        self.value = None

    def update(self, new_value):

        # 第一次直接初始化
        if self.value is None:
            self.value = new_value
            return int(self.value)

        # 低通滤波公式
        self.value = (
            self.alpha * self.value +
            (1 - self.alpha) * new_value
        )

        return int(self.value)