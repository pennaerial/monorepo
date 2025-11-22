import cv2 as cv
import numpy as np
from typing import Optional, Tuple


class ColorRectangleDetector:
    """
    Detects rectangles of a specific color in images using threshold and contour detection.
    """
    
    def __init__(
        self,
        target_color: Tuple[int, int, int] = (255, 255, 0),  # modify this based on color
        threshold_value: int = 150,
        min_area: int = 1000,
        max_area: int = 10000,
        blur_iterations: int = 4
    ):
        """
        Initialize the color rectangle detector.
        
        Args:
            target_color: Target color in HSV threshold space (default: yellow)
            threshold_value: Threshold value for binary threshold
            min_area: Minimum contour area to consider
            max_area: Maximum contour area to consider
            blur_iterations: Number of Gaussian blur passes
        """
        self.target_color = np.array(target_color)
        self.threshold_value = threshold_value
        self.min_area = min_area
        self.max_area = max_area
        self.blur_iterations = blur_iterations
        
    def detect_rectangle(self, img: np.ndarray) -> Optional[Tuple[int, int, float]]:
        """
        Detect a colored rectangle in the image and return its center.
        
        Args:
            img: BGR image from camera
            
        Returns:
            Tuple of (center_x, center_y, area) or None if no valid rectangle found
        """
        if img is None or img.size == 0:
            return None
        
        # Convert to HSV
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        # Apply Gaussian blur multiple times to decrease noise
        blur = hsv_img.copy()
        for _ in range(self.blur_iterations):
            blur = cv.GaussianBlur(blur, (5, 5), 0)
        
        # Use threshold function to filter out certain colors
        ret, thresh = cv.threshold(blur, self.threshold_value, 255, cv.THRESH_BINARY)
        
        # Filter for the target color
        # Create mask range around target color
        lower_bound = np.array([0, 0, 0])
        upper_bound = self.target_color
        mask = cv.inRange(thresh, lower_bound, upper_bound)
        
        # Use Canny edge detection
        edged = cv.Canny(mask, 30, 200)
        
        # Find contours
        contours, hierarchy = cv.findContours(
            edged.copy(), 
            cv.RETR_EXTERNAL, 
            cv.CHAIN_APPROX_NONE
        )
        
        # Find the largest valid contour
        valid_contours = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if self.min_area <= area <= self.max_area:
                valid_contours.append((cnt, area))
        
        if not valid_contours:
            return None
        
        # Get the largest valid contour
        largest_contour, area = max(valid_contours, key=lambda x: x[1])
        
        # Calculate center using moments
        M = cv.moments(largest_contour)
        
        if M['m00'] == 0:
            return None
        
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        
        return (cX, cY, area)
    
    def detect_and_visualize(self, img: np.ndarray) -> Tuple[np.ndarray, Optional[Tuple[int, int, float]]]:
        """
        Detect rectangle and return annotated image.
        
        Args:
            img: BGR image from camera
            
        Returns:
            Tuple of (annotated_image, detection_result)
        """
        result = self.detect_rectangle(img)
        annotated = img.copy()
        
        if result is None:
            return annotated, None
        
        cX, cY, area = result
        
        # Convert to HSV and process to get contours for visualization
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        blur = hsv_img.copy()
        for _ in range(self.blur_iterations):
            blur = cv.GaussianBlur(blur, (5, 5), 0)
        
        ret, thresh = cv.threshold(blur, self.threshold_value, 255, cv.THRESH_BINARY)
        mask = cv.inRange(thresh, np.array([0, 0, 0]), self.target_color)
        edged = cv.Canny(mask, 30, 200)
        contours, _ = cv.findContours(edged.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        
        # Draw the detected contour
        for cnt in contours:
            if self.min_area <= cv.contourArea(cnt) <= self.max_area:
                cv.drawContours(annotated, [cnt], -1, (0, 255, 0), 3)
        
        # Draw center point
        cv.circle(annotated, (cX, cY), 7, (255, 255, 255), -1)
        cv.circle(annotated, (cX, cY), 10, (0, 0, 255), 2)
        
        # Draw crosshair at image center for reference
        h, w = img.shape[:2]
        img_center_x, img_center_y = w // 2, h // 2
        cv.line(annotated, (img_center_x - 20, img_center_y), 
                (img_center_x + 20, img_center_y), (255, 0, 0), 2)
        cv.line(annotated, (img_center_x, img_center_y - 20), 
                (img_center_x, img_center_y + 20), (255, 0, 0), 2)
        
        # Add text with detection info
        cv.putText(annotated, f"Target: ({cX}, {cY})", 
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated, f"Area: {int(area)}", 
                   (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return annotated, result