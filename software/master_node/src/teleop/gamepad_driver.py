"""
Gamepad Driver — Reads wireless gamepad input for manual teleoperation.

Supports PS4 DualShock and Xbox controllers via pygame.
"""

import logging
import pygame

logger = logging.getLogger(__name__)


class GamepadDriver:
    """Reads gamepad axes and buttons for robot teleoperation."""

    def __init__(self, deadzone: float = 0.1):
        self.deadzone = deadzone
        self._joystick = None
        self._connected = False

        # Current state
        self.left_x = 0.0   # Left stick X (-1 to 1) -> strafe (vx)
        self.left_y = 0.0   # Left stick Y (-1 to 1) -> forward (vy)
        self.right_x = 0.0  # Right stick X (-1 to 1) -> rotation (omega)
        self.btn_a = False   # A/Cross -> switch to autonomous
        self.btn_b = False   # B/Circle -> emergency stop

    def init(self):
        """Initialize pygame joystick subsystem."""
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._connected = True
            logger.info(f"Gamepad connected: {self._joystick.get_name()}")
        else:
            logger.warning("No gamepad detected")
            self._connected = False

    def update(self):
        """Poll gamepad state. Call this in your main loop."""
        pygame.event.pump()

        if not self._connected:
            # Try to reconnect
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
                self._connected = True
                logger.info("Gamepad reconnected")
            return

        # Read axes (axis mapping may vary by controller)
        self.left_x = self._apply_deadzone(self._joystick.get_axis(0))
        self.left_y = self._apply_deadzone(-self._joystick.get_axis(1))  # Invert Y
        self.right_x = self._apply_deadzone(self._joystick.get_axis(3))

        # Read buttons
        self.btn_a = self._joystick.get_button(0)  # A / Cross
        self.btn_b = self._joystick.get_button(1)  # B / Circle

    def get_velocity_command(self, linear_scale: float = 0.3,
                             angular_scale: float = 1.0) -> tuple:
        """
        Get scaled velocity command from gamepad state.

        Returns:
            (vx, vy, omega) in m/s and rad/s.
        """
        vx = self.left_x * linear_scale
        vy = self.left_y * linear_scale
        omega = -self.right_x * angular_scale  # Negate so right = clockwise
        return vx, vy, omega

    def is_connected(self) -> bool:
        return self._connected

    def close(self):
        """Clean up pygame."""
        pygame.joystick.quit()
        pygame.quit()

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Rescale so output starts from 0 after deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
