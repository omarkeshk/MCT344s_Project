"""
Camera Driver — Manages camera capture and provides frames.
"""

import logging
import threading
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class CameraDriver:
    """Threaded camera capture for continuous frame access."""

    def __init__(self, device_id: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        self._cap = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = False
        self._thread = None

        # Camera calibration (load from file if available)
        self.camera_matrix = None
        self.dist_coeffs = None

    def open(self):
        """Open camera and start capture thread."""
        self._cap = cv2.VideoCapture(self.device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.device_id}")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        logger.info(f"Camera opened: device={self.device_id}, "
                     f"{self.width}x{self.height}@{self.fps}fps")

    def close(self):
        """Stop capture and release camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        logger.info("Camera closed")

    def get_frame(self) -> np.ndarray | None:
        """Get the latest captured frame (thread-safe)."""
        with self._frame_lock:
            if self._frame is not None:
                return self._frame.copy()
        return None

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        """Apply camera calibration if available."""
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        return frame

    def load_calibration(self, filepath: str):
        """Load camera calibration from a numpy .npz file."""
        try:
            data = np.load(filepath)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            logger.info(f"Loaded camera calibration from {filepath}")
        except Exception as e:
            logger.warning(f"Could not load calibration: {e}")

    def _capture_loop(self):
        """Background thread: continuously capture frames."""
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._frame_lock:
                    self._frame = frame
            else:
                time.sleep(0.01)
