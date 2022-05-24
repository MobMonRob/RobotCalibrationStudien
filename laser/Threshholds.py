
class Threshhold():

    def __init__(self, kernel_size, low_threshold, high_threshold, Hough_threshold, min_line_length, max_line_gap):
        self.kernel_size = kernel_size
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap
        self.Hough_threshold = Hough_threshold