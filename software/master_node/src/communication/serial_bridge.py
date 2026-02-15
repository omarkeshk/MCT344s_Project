"""
Serial Bridge — UART communication to ESP32 modules.

Manages two serial connections:
- Base controller (motors, odometry, IMU)
- Arm controller (servos, gripper, arm status)
"""

import threading
import time
import logging

import serial

from communication.protocol import (
    MsgId, parse_frame, encode_heartbeat,
    decode_odometry, decode_arm_status, decode_heartbeat, decode_sensor_data,
)

logger = logging.getLogger(__name__)


class SerialBridge:
    """Manages UART communication with one ESP32 module."""

    def __init__(self, port: str, baud_rate: int = 921600, name: str = "bridge"):
        self.port = port
        self.baud_rate = baud_rate
        self.name = name
        self._serial = None
        self._rx_buffer = bytearray()
        self._running = False
        self._rx_thread = None
        self._lock = threading.Lock()

        # Callbacks for received messages (msg_id -> callback)
        self._callbacks = {}

        # Last received data
        self.last_odometry = None
        self.last_arm_status = None
        self.last_sensor_data = None
        self.last_heartbeat_time = 0.0

    def register_callback(self, msg_id: int, callback):
        """Register a callback for a specific message ID."""
        self._callbacks[msg_id] = callback

    def open(self):
        """Open the serial connection."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.01,
            )
            self._running = True
            self._rx_thread = threading.Thread(
                target=self._rx_loop, daemon=True, name=f"{self.name}_rx"
            )
            self._rx_thread.start()
            logger.info(f"[{self.name}] Opened {self.port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            logger.error(f"[{self.name}] Failed to open {self.port}: {e}")
            raise

    def close(self):
        """Close the serial connection."""
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info(f"[{self.name}] Closed {self.port}")

    def send(self, frame: bytes):
        """Send a pre-built frame over serial."""
        if self._serial and self._serial.is_open:
            with self._lock:
                self._serial.write(frame)

    def send_heartbeat(self):
        """Send a heartbeat message."""
        timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF
        frame = encode_heartbeat(timestamp_ms, status=0)
        self.send(frame)

    def is_connected(self) -> bool:
        """Check if heartbeat has been received recently."""
        return (time.time() - self.last_heartbeat_time) < 1.0

    def _rx_loop(self):
        """Background thread: read serial data and parse frames."""
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting)
                    self._rx_buffer.extend(data)
                    self._process_buffer()
                else:
                    time.sleep(0.001)
            except serial.SerialException as e:
                logger.error(f"[{self.name}] Serial error: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        """Parse all complete frames from the RX buffer."""
        while True:
            msg_id, payload, consumed = parse_frame(self._rx_buffer)
            if consumed > 0:
                del self._rx_buffer[:consumed]
            if msg_id is None:
                break
            self._handle_message(msg_id, payload)

    def _handle_message(self, msg_id: int, payload: bytes):
        """Dispatch a received message."""
        try:
            if msg_id == MsgId.ODOMETRY:
                self.last_odometry = decode_odometry(payload)
            elif msg_id == MsgId.ARM_STATUS:
                self.last_arm_status = decode_arm_status(payload)
            elif msg_id == MsgId.HEARTBEAT:
                self.last_heartbeat_time = time.time()
                decode_heartbeat(payload)
            elif msg_id == MsgId.SENSOR_DATA:
                self.last_sensor_data = decode_sensor_data(payload)

            # Call registered callback if any
            if msg_id in self._callbacks:
                self._callbacks[msg_id](msg_id, payload)

        except Exception as e:
            logger.warning(f"[{self.name}] Error handling msg 0x{msg_id:02X}: {e}")
