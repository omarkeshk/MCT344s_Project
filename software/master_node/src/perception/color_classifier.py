"""
Color Classifier — HSV-based color detection as backup for QR code reading.
"""

import cv2
import numpy as np


# HSV color ranges (tuned for typical indoor lighting — may need calibration)
COLOR_RANGES = {
    'RED': [
        # Red wraps around in HSV, so we need two ranges
        {'lower': np.array([0, 100, 80]), 'upper': np.array([10, 255, 255])},
        {'lower': np.array([170, 100, 80]), 'upper': np.array([180, 255, 255])},
    ],
    'GREEN': [
        {'lower': np.array([35, 80, 80]), 'upper': np.array([85, 255, 255])},
    ],
    'BLUE': [
        {'lower': np.array([100, 100, 80]), 'upper': np.array([130, 255, 255])},
    ],
}

MIN_CONTOUR_AREA = 500  # Minimum pixels to consider a valid detection


class ColorClassifier:
    """Detects colored objects using HSV thresholding."""

    def __init__(self, color_ranges: dict = None):
        self.color_ranges = color_ranges or COLOR_RANGES

    def detect_color(self, frame: np.ndarray, roi: tuple = None) -> dict | None:
        """
        Detect the dominant color in the frame or ROI.

        Args:
            frame: BGR image from OpenCV.
            roi: Optional (x, y, w, h) region of interest.

        Returns:
            dict with 'color', 'center', 'area', 'contour' if found, else None.
        """
        if roi:
            x, y, w, h = roi
            region = frame[y:y+h, x:x+w]
        else:
            region = frame

        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

        best_match = None
        best_area = 0

        for color_name, ranges in self.color_ranges.items():
            # Create combined mask for this color
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for r in ranges:
                partial_mask = cv2.inRange(hsv, r['lower'], r['upper'])
                mask = cv2.bitwise_or(mask, partial_mask)

            # Clean up mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area > MIN_CONTOUR_AREA and area > best_area:
                    M = cv2.moments(largest)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        # Adjust for ROI offset
                        if roi:
                            cx += roi[0]
                            cy += roi[1]

                        best_match = {
                            'color': color_name,
                            'center': (cx, cy),
                            'area': area,
                            'contour': largest,
                        }
                        best_area = area

        return best_match

    def estimate_distance(self, contour_area: float, known_size_mm: float = 50.0,
                          focal_length_px: float = 600.0) -> float:
        """
        Estimate distance to object based on contour area.

        Args:
            contour_area: Area of detected contour in pixels.
            known_size_mm: Known real-world size of the object (cube side).
            focal_length_px: Camera focal length in pixels (needs calibration).

        Returns:
            Estimated distance in mm.
        """
        if contour_area <= 0:
            return float('inf')
        apparent_size = np.sqrt(contour_area)
        distance = (known_size_mm * focal_length_px) / apparent_size
        return distance

    def draw_detection(self, frame: np.ndarray, detection: dict) -> np.ndarray:
        """Draw color detection overlay."""
        annotated = frame.copy()

        color_bgr = {
            'RED': (0, 0, 255),
            'GREEN': (0, 255, 0),
            'BLUE': (255, 0, 0),
        }.get(detection['color'], (255, 255, 255))

        cv2.drawContours(annotated, [detection['contour']], -1, color_bgr, 2)

        cx, cy = detection['center']
        cv2.circle(annotated, (cx, cy), 5, color_bgr, -1)
        cv2.putText(annotated, detection['color'], (cx - 30, cy - 15),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)

        return annotated
