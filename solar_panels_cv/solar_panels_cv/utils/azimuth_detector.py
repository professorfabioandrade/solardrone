import cv2
import numpy as np
import math
from typing import Optional, Tuple, Callable

class AzimuthDetector:
    def __init__(self, info_print: Callable) -> None:
        self.image: Optional[np.ndarray] = None
        self.edges: Optional[np.ndarray] = None
        self.lines: Optional[np.ndarray] = None

        self.print_ = info_print

    def __call__(self, img_data: np.ndarray) -> Optional[Tuple[float, float]]:
        self.image = img_data
        self.edges = None
        self.lines = None

        self.detect_edges()
        self.detect_lines()
        tilt_angle, distance = self.find_horizontal_line_tilt()

        return (tilt_angle, distance)
        
    def detect_edges(self) -> None:
        if self.image is not None:
            self.edges = cv2.Canny(self.image, 50, 150, apertureSize=3)
        
    def detect_lines(self) -> None:
        if self.edges is not None:
            self.lines = cv2.HoughLinesP(self.edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)
        
    def find_horizontal_line_tilt(self) -> Optional[Tuple[float, float]]:
        if self.lines is None:
            return (None, None)
        
        # Calculate the middle of the image
        mid_y = self.image.shape[0] // 2
        
        # Find the horizontal line closest to the middle of the image
        closest_line = None
        min_distance = float('inf')
        
        for line in self.lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate the vertical distance from the middle of the image
            distance = abs((y1 + y2) / 2 - mid_y)
            
            if distance < min_distance:
                min_distance = distance
                closest_line = (x1, y1, x2, y2)
        
        if closest_line is None:
            self.print_("No horizontal line found")
            return (None, None)
        
        x1, y1, x2, y2 = closest_line
        
        # Calculate the tilt angle in degrees
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        
        return (angle, distance)
