import cv2
import numpy as np
import math
from typing import Optional, Tuple, Callable

class AzimuthDetector:
    def __init__(self, info_print: Callable) -> None:
        self.image: Optional[np.ndarray] = None
        self.edges: Optional[list[np.ndarray]] = None
        self.lines: Optional[list[np.ndarray]] = None

        self.print_ = info_print

    def __call__(self, img_data: np.ndarray) -> Optional[Tuple[float, float]]:
        self.image = img_data
        self.edges = None
        self.lines = None

        self.detect_edges()
        self.detect_lines()
        tilt_angle, distance, pos_line = self.find_horizontal_line_tilt()

        return (tilt_angle, distance, pos_line)
    
    @staticmethod
    def adaptative_threshold(img: np.ndarray) -> np.ndarray:
        # handle variations in illuminations
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        return cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 55, 2) 
    
    @staticmethod
    def morphological_operations(img: np.ndarray) -> np.ndarray:
        # close small gaps within rectangles
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
        closed = cv2.morphologyEx(img,cv2.MORPH_CLOSE, kernel_close)

        # remove internal noise
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (13,13))
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open)

        # close bigger gaps within rectangles
        final_kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (21,21))
        return cv2.morphologyEx(opened,cv2.MORPH_CLOSE, final_kernel_close)
           
    def detect_edges(self) -> None:
        if self.image is not None:
            thresh_image = self.adaptative_threshold(self.image)            
            final_img = self.morphological_operations(thresh_image)            
            self.edges = cv2.Canny(final_img, threshold1=100, threshold2=200, apertureSize=3)
        
    def detect_lines(self) -> None:
        if self.edges is not None:
            all_lines = cv2.HoughLinesP(self.edges, rho=1, theta=np.pi/180, threshold=150, minLineLength=150, maxLineGap=50)
            self.filter_lines(all_lines)
            
    def find_horizontal_line_tilt(self) -> Optional[Tuple[float, float, Tuple[int, int, int, int]]]:
        if self.lines is None:
            return (None, None, None)
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
        
        return (angle, distance, (int(x1), int(y1), int(x2), int(y2)))
    
    def filter_lines(self, lines: Optional[list[Tuple[float, float, float, float]]]) -> None:
        if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if abs(y2-y1) < abs(x2-x1):
                        if self.lines is None:
                            self.lines = []
                        self.lines.append(line)
