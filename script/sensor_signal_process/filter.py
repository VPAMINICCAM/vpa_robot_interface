class ComplementaryFilter:
    def __init__(self, alpha):
        """
        Initialize the complementary filter with a given alpha value.
        
        :param alpha: The filter coefficient (0 < alpha < 1)
        """
        if not (0 < alpha < 1):
            raise ValueError("Alpha must be between 0 and 1")
        self.alpha = alpha
        self.estimated_signal = 0

    def update(self, signal1, signal2):
        """
        Update the estimated signal using the complementary filter formula.
        
        :param signal1: The first signal (e.g., from an accelerometer)
        :param signal2: The second signal (e.g., from a gyroscope)
        :return: The estimated signal
        """
        self.estimated_signal = self.alpha * signal1 + (1 - self.alpha) * signal2
        return self.estimated_signal

# Example usage:
# filter = ComplementaryFilter(alpha=0.98)
# estimated_signal = filter.update(signal1, signal2)