class LowPassFilter:
    def __init__(self, alpha):
        if not (0.0 < alpha <= 1.0):
            raise ValueError("Alpha must be in (0, 1]")
        self.alpha = alpha
        self.filtered_value = None
        self.initialized = False

    def filter(self, value):
        if not self.initialized:
            self.filtered_value = value
            self.initialized = True
        else:
            self.filtered_value = self.alpha * value + (1.0 - self.alpha) * self.filtered_value
        return self.filtered_value

    def reset(self, value=None):
        self.filtered_value = value
        self.initialized = value is not None