"""
QR Code Detector — Reads QR codes from camera frames to identify cube colors.
"""

import logging
import cv2
import numpy as np
from pyzbar import pyzbar

logger = logging.getLogger(__name__)


class QRDetector:
    """Detects and decodes QR codes in camera frames."""

    def __init__(self):
        self.last_detection = None
        self.last_bbox = None
        self._valid_colors = {'RED', 'BLUE', 'GREEN'}

    def detect(self, frame: np.ndarray) -> dict | None:
        """
        Detect and decode QR code in the given frame.

        Args:
            frame: BGR image from OpenCV.

        Returns:
            dict with 'color', 'data', 'bbox', 'center' if QR found, else None.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded_objects = pyzbar.decode(gray)

        for obj in decoded_objects:
            data = obj.data.decode('utf-8').strip().upper()
            points = obj.polygon

            if len(points) >= 4:
                bbox = np.array([(p.x, p.y) for p in points], dtype=np.int32)
                center_x = int(np.mean([p.x for p in points]))
                center_y = int(np.mean([p.y for p in points]))
            else:
                rect = obj.rect
                bbox = np.array([
                    [rect.left, rect.top],
                    [rect.left + rect.width, rect.top],
                    [rect.left + rect.width, rect.top + rect.height],
                    [rect.left, rect.top + rect.height],
                ], dtype=np.int32)
                center_x = rect.left + rect.width // 2
                center_y = rect.top + rect.height // 2

            color = self._parse_color(data)
            if color:
                result = {
                    'color': color,
                    'data': data,
                    'bbox': bbox,
                    'center': (center_x, center_y),
                }
                self.last_detection = result
                self.last_bbox = bbox
                logger.info(f"QR detected: color={color}, data='{data}'")
                return result

        return None

    def _parse_color(self, data: str) -> str | None:
        """Extract color from QR data string."""
        # Try direct match
        if data in self._valid_colors:
            return data

        # Try finding color keyword in the data
        for color in self._valid_colors:
            if color in data:
                return color

        logger.warning(f"QR decoded but unrecognized color: '{data}'")
        return None

    def draw_detection(self, frame: np.ndarray, detection: dict) -> np.ndarray:
        """Draw QR detection overlay on the frame."""
        annotated = frame.copy()

        # Draw bounding polygon
        color_bgr = {
            'RED': (0, 0, 255),
            'GREEN': (0, 255, 0),
            'BLUE': (255, 0, 0),
        }.get(detection['color'], (255, 255, 255))

        cv2.polylines(annotated, [detection['bbox']], True, color_bgr, 2)

        # Draw center point
        cx, cy = detection['center']
        cv2.circle(annotated, (cx, cy), 5, color_bgr, -1)

        # Draw label
        label = f"{detection['color']}"
        cv2.putText(annotated, label, (cx - 30, cy - 15),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)

        return annotated
